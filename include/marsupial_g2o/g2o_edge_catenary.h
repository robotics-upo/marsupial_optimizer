#pragma once

#ifndef G2O_CATENARY_EDGE_H
#define G2O_CATENARY_EDGE_H


#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/config.h"
#include "g2o/core/base_binary_edge.h"
#include <vector>

// #include <pcl/search/kdtree.h>

#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>

#include "marsupial_g2o/nearNeighbor.hpp"
#include "marsupial_g2o/bisection_catenary_3D.h"

#include "g2o_vertex_catenary_length.h"

#define PRINTF_GREEN "\x1B[32m"
#define PRINTF_BLUE "\x1B[34m"
#define PRINTF_REGULAR "\x1B[0m"
#define PRINTF_RED "\x1B[31m"

#include <stdio.h>

namespace g2o {


class G2OCatenaryEdge : public BaseBinaryEdge<3, vector<double>, VertexPointXYZ, VertexCatenaryLength>
{
	public:
		G2OCatenaryEdge(ros::NodeHandlePtr nhP);
        
		struct points_obs_close_cat
        {
            Eigen::Vector3d obs;
            Eigen::Vector3d cat;
        };

		double _length_catenary;
		// double _porcentage = 1.1;

		std::vector<geometry_msgs::Point> _points_catenary;
		visualization_msgs::MarkerArray _catenary_marker;
		ros::Publisher catenary_marker_pub_;

		bisectionCatenary bc;

		void markerPoints(visualization_msgs::MarkerArray _marker, std::vector<geometry_msgs::Point> _vector, int _suffix, int _n_v);	// Print the catenary between UGV and vertex 
		void clearMarkers(visualization_msgs::MarkerArray _marker,auto _s);

		int _id_marker;		// Related with Marker frame.id
		int _n_vertices;	// Number of Vertices that no are fixed
		int _prev_size_marker = 0; // Save the previus size value of the catenary for vertex optimized To delete marker 

		NearNeighbor _nn;
		Eigen::Vector3d _obstacles_near_catenary;
		pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN;
		pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points;
		double _radius; 
		double _radius_security; 

		double _d_curr, _x, _y, _z;
		bool _debug;
		double approximation_;
		double _length_cat_;
		double bound_bisection_a,bound_bisection_b;

		double bound_z_negative = 0.1;
		double lower_point_cat_z;

		geometry_msgs::Point _initial_pos_cat_ugv;

		int n_points_cat_dis;
		double _d_cat_obs_prev;
		
		double sum_error_per_cat;
		double factor_error_straight_catenary;
		double factor_negative_z;
		double factor_error_0;

		void computeError()
		{
    		const VertexPointXYZ * pose = static_cast<const VertexPointXYZ*> (_vertices[0]);
    		const VertexCatenaryLength * l_catenary = static_cast<const VertexCatenaryLength*> (_vertices[1]);
	
			lower_point_cat_z = bound_z_negative;

			// Calculate Catenary
			_x = (pose->estimate().x()-_initial_pos_cat_ugv.x);
			_y = (pose->estimate().y()-_initial_pos_cat_ugv.y);
			_z = (pose->estimate().z()-_initial_pos_cat_ugv.z);
			_d_curr = sqrt( _x * _x + _y * _y + _z * _z); // Distance UGV and Vertex

			if (l_catenary->estimate() > _d_curr)
				_length_catenary = l_catenary->estimate();
			else
				_length_catenary = _d_curr*1.001;

			if (_length_catenary > 100.0 || pose->estimate().z() < bound_z_negative){
				ROS_ERROR ("OUT OF RANGE: Not posible to get Catenary for vertex[%i] = [%f %f %f] and length_catenary =[%f]", pose->id(),pose->estimate().x(),pose->estimate().y(),pose->estimate().z(),_length_catenary);
				_error[0] = 100000000.0 * (pose->estimate().z() - bound_z_negative); 
				_error[1] = 100000000.0 * (pose->estimate().z() - bound_z_negative);
				_error[2] = 100000000.0 * (pose->estimate().z() - bound_z_negative);
			}
			else{
				bc.setNumberPointsCatenary(_length_catenary*10.0);
				bc.setFactorBisection(bound_bisection_a,bound_bisection_b);
				bc.configBisection(_length_catenary,_initial_pos_cat_ugv.x,_initial_pos_cat_ugv.y,_initial_pos_cat_ugv.z,pose->estimate().x(),pose->estimate().y(),pose->estimate().z(),pose->id(),"pose");

				_points_catenary.clear();
				bc.getPointCatenary3D(_points_catenary);

				if (_points_catenary.size()<1.0){
					ROS_ERROR ("Not posible to get Catenary for vertex[%i] = [%f %f %f]", pose->id(),pose->estimate().x(),pose->estimate().y(),pose->estimate().z());
				}
				
				_id_marker = pose->id();
				if (_prev_size_marker >= _points_catenary.size() )
					clearMarkers(_catenary_marker,_prev_size_marker);
				else
					clearMarkers(_catenary_marker,_points_catenary.size());
				
				markerPoints(_catenary_marker ,_points_catenary,_id_marker,_n_vertices);
				_prev_size_marker = _points_catenary.size();

				int count_negative_z = 0;

				sum_error_per_cat = 0.0;
				factor_error_straight_catenary = 1.0;
				_d_cat_obs_prev = 10000.0;

				factor_negative_z = 0.0;
				factor_error_0 = 1.0;

				n_points_cat_dis = ceil(1.5*ceil(_length_catenary)); // parameter to ignore collsion points in the begining and in the end of catenary
				if (n_points_cat_dis < 5)
					n_points_cat_dis = 5;

				// Analysis each point Catenary: To check distance points to obstacle and state pointer below -z axes
				Eigen::Vector3d p_cat;
				for (size_t i = 0 ; i < _points_catenary.size() ; i++){
					p_cat.x() = _points_catenary[i].x;
					p_cat.y() = _points_catenary[i].y;
					p_cat.z() = _points_catenary[i].z;
					
					_obstacles_near_catenary = _nn.nearestObstacleVertex(kdT_From_NN , p_cat,obstacles_Points);
					double _d_cat_obs = (p_cat-_obstacles_near_catenary).norm();

					if (_d_cat_obs < _radius && (i > n_points_cat_dis ) && (i < _points_catenary.size()-n_points_cat_dis/2)){
						sum_error_per_cat = 100000.0 * exp(_radius -_d_cat_obs) + sum_error_per_cat;
						// factor_error_straight_catenary = 0.0;
						// if(pose->id() == 20 || pose->id() == 19 || pose->id() == 18)
						// 	printf("_d_cat_obs < _radius vertex=[%i] sum_error_per_cat=[%f]\n",pose->id(),sum_error_per_cat);
					}

					if(_obstacles_near_catenary.z() > p_cat.z() && _d_cat_obs < _radius +0.15 && (i > n_points_cat_dis ) && (i < _points_catenary.size()-n_points_cat_dis/2)){
						factor_error_straight_catenary = 0.0;
					}
					// Analysis II: To check if there are catenary points above z= 0.0
					if (p_cat.z() < bound_z_negative){
						count_negative_z++;
						factor_error_0 = 0.0;
						factor_error_straight_catenary = 1.0;
						factor_negative_z = 1.0;
						if ( p_cat.z() < lower_point_cat_z)
							lower_point_cat_z = p_cat.z();
					// printf("p_cat.z() < bound_z_negative vertex=[%i]\n",pose->id());
					}
				}
		
				// Computed Error[0]: Collision catenary with obstacle
				// Computed Error[1]: To straight the Cable
				// Computed Error[2]: To Avoid -z valus for catenary
				_error[0] =  sum_error_per_cat;
				_error[1] = 10.0*(_length_catenary -  _d_curr)*factor_error_straight_catenary; 
				// _error[2] = 100000000.0*(double)(count_negative_z)*_length_catenary;
				_error[2] = 10000000.0*exp(10.0*(bound_z_negative + sqrt(pow(bound_z_negative - p_cat.z(),2))) ) * factor_negative_z;
				// if(pose->id() == 10){
				// 	printf("Vertex=[%i] , Error=[%f %f %f] length_catenary=[%f]\n",pose->id(),_error[0],_error[1],_error[2],_length_catenary);
				// }
			}
		}

		virtual bool read(std::istream& is);

		virtual bool write(std::ostream& os) const;

		inline void readKDTree(const pcl::KdTreeFLANN <pcl::PointXYZ> &kdT_, pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_){
			kdT_From_NN = kdT_;
			obstacles_Points = o_p_;
		}

		//Take the position value of the UGV
		virtual void setMeasurement(const vector<double> _p) {_measurement = _p;}
		
		virtual void numberVerticesNotFixed(const int& _n_v) {_n_vertices = _n_v;}

		inline void setRadius(const double& _s_r){_radius = _s_r;}

		inline void setInitialPosCatUGV(const geometry_msgs::Point& _p) {_initial_pos_cat_ugv = _p;}

		// inline void setLengthCatenary(double _v){_length_catenary = _v;} // Set the catenary length related with distance between UGV and vertex

		inline void setBoundForBisection(double _fa,double _fb){
			bound_bisection_a = _fa; 
			bound_bisection_b = _fb;
		}


	protected: 
};
}

#endif