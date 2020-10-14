#pragma once

#ifndef G2O_CATENARY_EDGE_H
#define G2O_CATENARY_EDGE_H


#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/config.h"
#include "g2o/core/base_multi_edge.h"
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


class G2OCatenaryEdge : public BaseMultiEdge<4, vector<double>>
{
	public:
		G2OCatenaryEdge(ros::NodeHandlePtr nhP);
        
		struct points_obs_close_cat
        {
            Eigen::Vector3d obs;
            Eigen::Vector3d cat;
        };

		double _length_catenary1;
		double _length_catenary2;
		// double _porcentage = 1.1;

		std::vector<geometry_msgs::Point> _points_catenary1;
		std::vector<geometry_msgs::Point> _points_catenary2;
		visualization_msgs::MarkerArray _catenary_marker;
		ros::Publisher catenary_marker_pub_;

		bisectionCatenary bc1;
		bisectionCatenary bc2;

		void markerPoints(visualization_msgs::MarkerArray _marker, std::vector<geometry_msgs::Point> _vector, int _suffix, int _n_v);	// Print the catenary between UGV and vertex 
		void clearMarkers(visualization_msgs::MarkerArray _marker,auto _s);

		int _id_marker1;		// Related with Marker frame.id
		int _n_vertices;	// Number of Vertices that no are fixed
		int _prev_size_marker = 0; // Save the previus size value of the catenary for vertex optimized To delete marker 

		NearNeighbor _nn;
		Eigen::Vector3d _obstacles_near_catenary;
		pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN;
		pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points;
		double _radius; 
		double _radius_security; 

		double _d_curr1,_d_curr2, _x, _y, _z;
		bool _debug;
		double approximation_;
		double _length_cat_;
		double bound_bisection_a,bound_bisection_b;

		double bound_z_negative = 0.1;
		double lower_point_cat_z;

		geometry_msgs::Point _initial_pos_cat_ugv;

		int n_points_cat_dis;
		double _d_cat_obs_prev;
		
		double sum_error_per_cat_0;
		double factor_error_straight_catenary;
		std::vector<geometry_msgs::Point> v_factor1;
		std::vector<geometry_msgs::Point> v_factor2;
		std::vector<Eigen::Vector3d> _vector_points_line;
		int num_points;
		double sum_error_per_cat_3;
		int _id_marker2;		// Related with Marker frame.id

		int aux_id_point_c1, aux_id_point_c2;
		void computeError()
		{
    		const VertexPointXYZ * pose1 = static_cast<const VertexPointXYZ*> (_vertices[0]);
    		const VertexPointXYZ * pose2 = static_cast<const VertexPointXYZ*> (_vertices[1]);
    		const VertexCatenaryLength * l_catenary1 = static_cast<const VertexCatenaryLength*> (_vertices[2]);
    		const VertexCatenaryLength * l_catenary2 = static_cast<const VertexCatenaryLength*> (_vertices[3]);
	
			lower_point_cat_z = bound_z_negative;

			// Calculate Catenary 1
			_x = (pose1->estimate().x()-_initial_pos_cat_ugv.x);
			_y = (pose1->estimate().y()-_initial_pos_cat_ugv.y);
			_z = (pose1->estimate().z()-_initial_pos_cat_ugv.z);
			_d_curr1 = sqrt( _x * _x + _y * _y + _z * _z); // Distance UGV and Vertex

			// Calculate Catenary 2
			_x = (pose2->estimate().x()-_initial_pos_cat_ugv.x);
			_y = (pose2->estimate().y()-_initial_pos_cat_ugv.y);
			_z = (pose2->estimate().z()-_initial_pos_cat_ugv.z);
			_d_curr2 = sqrt( _x * _x + _y * _y + _z * _z); // Distance UGV and Vertex

			if (l_catenary1->estimate() > _d_curr1)
				_length_catenary1 = l_catenary1->estimate();
			else
				_length_catenary1 = _d_curr1*1.001;
			if (l_catenary2->estimate() > _d_curr2)
				_length_catenary2 = l_catenary2->estimate();
			else
				_length_catenary2 = _d_curr2*1.001;


			if (_length_catenary1 > 100.0 || pose1->estimate().z() < bound_z_negative || _length_catenary2 > 100.0 || pose2->estimate().z() < bound_z_negative){
				ROS_ERROR ("OUT OF RANGE: Not posible to get Catenary vertex[%i]=[%f %f %f] length =[%f] and vertex[%i]=[%f %f %f] length=[%f]",
				pose1->id(),pose1->estimate().x(),pose1->estimate().y(),pose1->estimate().z(),_length_catenary1,
				pose2->id(),pose2->estimate().x(),pose2->estimate().y(),pose2->estimate().z(),_length_catenary2);
				_error[0] = 100000000.0 * sqrt(pow(pose1->estimate().z() - bound_z_negative,2)) + 100000000.0 * sqrt(pow(pose2->estimate().z() - bound_z_negative,2)); 
				_error[1] = 100000000.0 * sqrt(pow(pose1->estimate().z() - bound_z_negative,2)) + 100000000.0 * sqrt(pow(pose2->estimate().z() - bound_z_negative,2));
				_error[2] = 100000000.0 * sqrt(pow(pose1->estimate().z() - bound_z_negative,2)) + 100000000.0 * sqrt(pow(pose2->estimate().z() - bound_z_negative,2));
			}
			else{
				bc1.setNumberPointsCatenary(_length_catenary1*10.0);
				bc1.setFactorBisection(bound_bisection_a,bound_bisection_b);
				bc1.configBisection(_length_catenary1,_initial_pos_cat_ugv.x,_initial_pos_cat_ugv.y,_initial_pos_cat_ugv.z,pose1->estimate().x(),pose1->estimate().y(),pose1->estimate().z(),pose1->id(),"pose1");
				_points_catenary1.clear();
				bc1.getPointCatenary3D(_points_catenary1);

				bc2.setNumberPointsCatenary(_length_catenary2*10.0);
				bc2.setFactorBisection(bound_bisection_a,bound_bisection_b);
				bc2.configBisection(_length_catenary2,_initial_pos_cat_ugv.x,_initial_pos_cat_ugv.y,_initial_pos_cat_ugv.z,pose2->estimate().x(),pose2->estimate().y(),pose2->estimate().z(),pose2->id(),"pose2");
				_points_catenary2.clear();
				bc2.getPointCatenary3D(_points_catenary2);


				if (_points_catenary1.size()<1.0){
					ROS_ERROR ("G2OCatenaryEdge computeError(): Not posible to get Catenary1 for vertex[%i] = [%f %f %f]", pose1->id(),pose1->estimate().x(),pose1->estimate().y(),pose1->estimate().z());
				}
				if (_points_catenary2.size()<1.0){
					ROS_ERROR ("G2OCatenaryEdge computeError(): Not posible to get Catenary2 for vertex[%i] = [%f %f %f]", pose2->id(),pose2->estimate().x(),pose2->estimate().y(),pose2->estimate().z());
				}
				
				_id_marker1 = pose1->id();
				if (_prev_size_marker >= _points_catenary1.size() )
					clearMarkers(_catenary_marker,_prev_size_marker);
				else
					clearMarkers(_catenary_marker,_points_catenary1.size());
				
				markerPoints(_catenary_marker ,_points_catenary1,_id_marker1,_n_vertices);
				_prev_size_marker = _points_catenary1.size();

				// if (pose2->id() == _measurement.size()){
					_id_marker2 = pose2->id();
					if (_prev_size_marker >= _points_catenary2.size() )
						clearMarkers(_catenary_marker,_prev_size_marker);
					else
						clearMarkers(_catenary_marker,_points_catenary2.size());
					
					markerPoints(_catenary_marker ,_points_catenary2,_id_marker2,_n_vertices);
					_prev_size_marker = _points_catenary2.size();
				// }


				int count_negative_z = 0;

				sum_error_per_cat_0 = 0.0;
				factor_error_straight_catenary = 1.0;
				_d_cat_obs_prev = 10000.0;

				n_points_cat_dis = ceil(1.5*ceil(_length_catenary1)); // parameter to ignore collsion points in the begining and in the end of catenary
				if (n_points_cat_dis < 5)
					n_points_cat_dis = 5;

				// I. Analysis each point Catenary: To check distance catenary points to obstacle and state pointer below -z axes
				for (size_t i = 0 ; i < _points_catenary1.size() ; i++){
					Eigen::Vector3d p_cat;
					p_cat.x() = _points_catenary1[i].x;
					p_cat.y() = _points_catenary1[i].y;
					p_cat.z() = _points_catenary1[i].z;
					
					_obstacles_near_catenary = _nn.nearestObstacleVertex(kdT_From_NN , p_cat,obstacles_Points);
					double _d_cat_obs = (p_cat-_obstacles_near_catenary).norm();

					if (_d_cat_obs < _radius && (i > n_points_cat_dis )){
						sum_error_per_cat_0 = 100000.0 * exp(_radius -_d_cat_obs_prev) + sum_error_per_cat_0;
						factor_error_straight_catenary = 0.0;
					}

					// Analysis II: To check if there are catenary points above z= 0.0
					if (p_cat.z() < bound_z_negative){
						count_negative_z++;
						if ( p_cat.z() < lower_point_cat_z)
							lower_point_cat_z = p_cat.z();
					}
				}
				
				// II. Analysis between Catenaries: To check between two catenaries
				double rate;
				string equal_status;
				if (_points_catenary1.size() <= _points_catenary2.size()){
					v_factor1 = _points_catenary1;
					v_factor2 = _points_catenary2;	
					equal_status ="catenary1.size() <= catenary2.size()";
				}
				else{
					v_factor1 = _points_catenary2;	
					v_factor2 = _points_catenary1;	
					equal_status ="catenary2.size() < catenary1.size()";
				}
				
				double v_f1_size =(double)v_factor1.size();
				double v_f2_size =(double)v_factor2.size();
				rate = v_f1_size/v_f2_size;

				sum_error_per_cat_3 = 0.0;	
				for (size_t i=0; i< v_factor2.size(); i++){
					int id_f2_to_f1 = ceil((i+1)*rate) -1 ;
					Eigen::Vector3d _pc1, _pc2;
					aux_id_point_c2= i;
					aux_id_point_c1 = id_f2_to_f1;
					_pc1.x() = v_factor1[id_f2_to_f1].x;
					_pc1.y() = v_factor1[id_f2_to_f1].y;
					_pc1.z() = v_factor1[id_f2_to_f1].z;
					_pc2.x() = v_factor2[i].x;
					_pc2.y() = v_factor2[i].y;
					_pc2.z() = v_factor2[i].z;
					double dist_pc2_pc1 = (_pc2 - _pc1).norm();
					num_points = (int)(ceil(dist_pc2_pc1 * 5.0));
					_vector_points_line.clear();
					straightTrajectoryVertices(_pc2.x(),_pc2.y(),_pc2.z(),_pc1.x(),_pc1.y(),_pc1.z(),num_points,_vector_points_line);
					ROS_ERROR("rate=[%f] size v_f2=[%lu/%lu] v_f1=[%i/%lu] equal_status=[%s] num_points=[%i] vector_points_line.size=[%lu]",rate,i,v_factor2.size(),id_f2_to_f1,v_factor1.size(),equal_status.c_str(),num_points,_vector_points_line.size());

					for (size_t j =0; j <_vector_points_line.size();j++){

						Eigen::Vector3d _near = _nn.nearestObstacleVertex(kdT_From_NN , _vector_points_line[j],obstacles_Points);

						double dist_straight_line_obstacle = (_vector_points_line[j] -_near ).norm();
						double _radius_cat_straight = 0.1;
						double n_points_cat_dis1 = ceil(1.5*ceil(_length_catenary1)); // parameter to ignore collsion points in the begining and in the end of catenary
						if (n_points_cat_dis1 < 5)
							n_points_cat_dis1 = 5;
						double n_points_cat_dis2 = ceil(1.5*ceil(_length_catenary2)); // parameter to ignore collsion points in the begining and in the end of catenary
						if (n_points_cat_dis2 < 5)
							n_points_cat_dis2 = 5;
						// printf("n_points_cat_dis1=[%f] n_points_cat_dis2=[%f]\n",n_points_cat_dis1,n_points_cat_dis2);
						if (dist_straight_line_obstacle < _radius_cat_straight && (id_f2_to_f1 > n_points_cat_dis1) && (i > n_points_cat_dis2)){
							sum_error_per_cat_3 = 100000.0 * exp(_d_curr2 -_length_catenary2) + sum_error_per_cat_3;
							sum_error_per_cat_0 = 0.0;
							printf("G2OCatenaryEdge computeError() vertex=[%i-%i] straight_line=[%lu] points=[%lu/%lu-%i/%lu] equal_status=[%s]\n",pose1->id(),pose2->id(),j,i,v_factor2.size(),id_f2_to_f1,v_factor1.size(),equal_status.c_str());
						}
					}
				}

		
				// Computed Error[0]: Collision catenary with obstacle
				// Computed Error[1]: To straight the Cable
				// Computed Error[2]: To Avoid -z valus for catenary
				_error[0] =  sum_error_per_cat_0;
				_error[1] = 2.0*(_length_catenary1 -  _d_curr1)*factor_error_straight_catenary; 
				_error[2] = 100000000.0*(double)(count_negative_z)*_length_catenary1;
				_error[3] = sum_error_per_cat_3;
				printf("vertex=[%i-%i] Error[0] = %f  Error[1] = %f  Error[2] = %f  Error[3] = %f\n",pose1->id(),pose2->id(),_error[0],_error[1],_error[2],_error[3]);
				// if (_error[3] > 0.0)
					// printf("G2OCatenaryEdge computeError() error[3]=[%f] vertex=[%i-%i] points_cat=[%i/%lu-%i/%lu] equal_status=[%s]\n",_error[3],pose1->id(),pose2->id(),aux_id_point_c1,v_factor1.size(),aux_id_point_c2,v_factor2.size(),equal_status.c_str());
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

		// inline void setLengthCatenary(double _v){_length_catenary1 = _v;} // Set the catenary length related with distance between UGV and vertex

		inline void setBoundForBisection(double _fa,double _fb){
			bound_bisection_a = _fa; 
			bound_bisection_b = _fb;
		}

		void straightTrajectoryVertices(double x1, double y1, double z1, double x2, double y2, double z2, int _n_v_u, std::vector<Eigen::Vector3d> &_v)
		{
			double _x, _y, _z;
			double _d= sqrt(pow(x2-x1,2)+pow(y2-y1,2)+pow(z2-z1,2));
			// double _n = ceil(_d*n_v_u_);
			double distance_xy = sqrt(pow(x2-x1,2)+pow(y2-y1,2));
			double distance_xz = sqrt(pow(x2-x1,2)+pow(z2-z1,2));
			if (distance_xy < 0.00001)
				distance_xy = 0.00001;
			if (distance_xz < 0.00001)
				distance_xz = 0.00001;
	
			double interval_xy = distance_xy/_n_v_u;		// Size of the interval between points in axes xy
			double interval_xz = distance_xz/_n_v_u;		// Size of the interval between points in axes xz
			for(int i = 1 ; i< _n_v_u+1 ; i++)
			{
				_x = x1 + i*interval_xy * ((x2-x1)/distance_xy);
				_y = y1 + i*interval_xy * ((y2-y1)/distance_xy);
				_z = z1 + i*interval_xz * ((z2-z1)/distance_xz);
			
				_v.push_back(Eigen::Vector3d(_x,_y,_z));
			}
		}


	protected: 
};
}

#endif