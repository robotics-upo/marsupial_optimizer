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
		Eigen::Vector3d obstacles_;
		Eigen::Vector3d prev_point_catenary;
		pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN;
		pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points;
		double _radius ;

		double _d_curr, _x, _y, _z;
		bool _debug;
		double approximation_;
		double _length_cat_;
		int mode_obstacle;
		double bound_bisection_a,bound_bisection_b;

		double bound_z_negative = 0.1;
		double lower_point_cat_z;

		geometry_msgs::Point _initial_pos_ugv;
	

		std::vector<points_obs_close_cat> _v_obs_collision_cat;
		std::vector<points_obs_close_cat> _v_obs_close_cat;
		// std::vector<Eigen::Vector3d> _v_p_cat_collision;

		void computeError()
		{
    		const VertexPointXYZ * pose = static_cast<const VertexPointXYZ*> (_vertices[0]);
    		const VertexCatenaryLength * l_catenary = static_cast<const VertexCatenaryLength*> (_vertices[1]);
	
			lower_point_cat_z = bound_z_negative;

			// Calculate Catenary
			_x = (pose->estimate().x()-_initial_pos_ugv.x);
			_y = (pose->estimate().y()-_initial_pos_ugv.y);
			_z = (pose->estimate().z()-_initial_pos_ugv.z);
			_d_curr = sqrt( _x * _x + _y * _y + _z * _z); // Distance UGV and Vertex


			if (l_catenary->estimate() > _d_curr)
				_length_catenary = l_catenary->estimate();
			else
				_length_catenary = _d_curr*1.001;
			bc.setNumberPointsCatenary(_length_catenary*10.0);
			bc.setFactorBisection(bound_bisection_a,bound_bisection_b);
			bc.configBisection(_length_catenary,_initial_pos_ugv.x,_initial_pos_ugv.y,_initial_pos_ugv.z,pose->estimate().x(),pose->estimate().y(),pose->estimate().z(),pose->id());

			_points_catenary.clear();
			// printf(PRINTF_RED);
			bc.getPointCatenary3D(_points_catenary);

			if (_points_catenary.size()<1.0){
				ROS_ERROR ("Not posible to get Catenary for vertex[%i] = [%f %f %f]", pose->id(),pose->estimate().x(),pose->estimate().y(),pose->estimate().z());
			}
			
			_id_marker = pose->id();
			// Print Catenary Marker between each vertex and UGV
			if (_prev_size_marker >= _points_catenary.size() )
				clearMarkers(_catenary_marker,_prev_size_marker);
			else
				clearMarkers(_catenary_marker,_points_catenary.size());
			
			markerPoints(_catenary_marker ,_points_catenary,_id_marker,_n_vertices);
			_prev_size_marker = _points_catenary.size();

			// To Obtain KdTree for each point in every calculated catenary 
			int count = 0 ; 
			int count_negative_z = 0;
			bool first_point_cat = true;
			bool inside_x, inside_y, inside_z;
			_v_obs_collision_cat.clear();
			// _v_p_cat_collision.clear();
			_v_obs_close_cat.clear();

			points_obs_close_cat _points1, _points2;

			for (size_t i = 0 ; i < _points_catenary.size() ; i++){
				Eigen::Vector3d p_cat;
				p_cat.x() = _points_catenary[i].x;
				p_cat.y() = _points_catenary[i].y;
				p_cat.z() = _points_catenary[i].z;
				
				_obstacles_near_catenary = _nn.nearestObstacleVertex(kdT_From_NN , p_cat,obstacles_Points);
				double _d_cat_obs = (p_cat-_obstacles_near_catenary).norm();
				

				// Get Data I Analysis Catenary: To check if there is obstacle between two catenary points
				int n_points_cat_dis = 4;
				if (_d_cat_obs < _radius && (i > n_points_cat_dis || i < (_points_catenary.size()- n_points_cat_dis/2 - 1))){
					_points1.obs = _obstacles_near_catenary;
					_points1.cat = p_cat;
					_v_obs_close_cat.push_back(_points1);
					printf("vertex[%i] _d=[%f] pos_point_cat =[%lu / %lu] _length_catenary=[%f]  _points1.cat =[%f %f %f] _points1.obs = [%f %f %f]\n",pose->id(),_d_cat_obs,i,_points_catenary.size(),_length_catenary,_points1.cat.x(),_points1.cat.y(),_points1.cat.z(),_points1.obs.x(),_points1.obs.y(),_points1.obs.z());
				}
				

				// double _d_points_cat = (p_cat-prev_point_catenary).norm();
				// approximation_ = _d_points_cat;
				
				// Get Data II Analysis Catenary: To check if there is obstacle between two catenary points
				approximation_ = _radius;
				inside_x = inside_y = inside_z = false;
				if (!first_point_cat){
					if (prev_point_catenary.x() < p_cat.x()){
						if ((prev_point_catenary.x() - approximation_) < _obstacles_near_catenary.x() && (p_cat.x() + approximation_) > _obstacles_near_catenary.x()){
							inside_x = true;
							obstacles_.x()=_obstacles_near_catenary.x();					
						}
					}
					else if (prev_point_catenary.x() > p_cat.x()){
						if ((prev_point_catenary.x() + approximation_) > _obstacles_near_catenary.x() && (p_cat.x() - approximation_) < _obstacles_near_catenary.x()){
							inside_x = true;
							obstacles_.x()=_obstacles_near_catenary.x();					
						}
					}
					else if (prev_point_catenary.x() == p_cat.x()){
						if ((prev_point_catenary.x() + approximation_) > _obstacles_near_catenary.x() && (prev_point_catenary.x() - approximation_) < _obstacles_near_catenary.x()){
							inside_x = true;
							obstacles_.x()=_obstacles_near_catenary.x();					
						}
					}
					else
						inside_x = false;

					if (prev_point_catenary.y() < p_cat.y()){
						if ((prev_point_catenary.y() - approximation_) < _obstacles_near_catenary.y() && (p_cat.y() + approximation_) > _obstacles_near_catenary.y()){
							inside_y = true;
							obstacles_.y()=_obstacles_near_catenary.y();					
						}
					}
					else if (prev_point_catenary.y() > p_cat.y()){
						if ((prev_point_catenary.y() + approximation_) > _obstacles_near_catenary.y() &&  (p_cat.y() - approximation_) < _obstacles_near_catenary.y()){
							inside_y = true;
							obstacles_.y()=_obstacles_near_catenary.y();					
						}
					}
					else if (prev_point_catenary.y() == p_cat.y()){
						if ((prev_point_catenary.y() + approximation_) > _obstacles_near_catenary.y() && (prev_point_catenary.y() - approximation_) < _obstacles_near_catenary.y()){
							inside_y = true;
							obstacles_.y()=_obstacles_near_catenary.y();					
						}
					}
					else
						inside_y = false;

					if (prev_point_catenary.z() < p_cat.z()){
						if ((prev_point_catenary.z() - approximation_) < _obstacles_near_catenary.z() && (p_cat.z() + approximation_) > _obstacles_near_catenary.z()){
							inside_z = true;
							obstacles_.z()=_obstacles_near_catenary.z();					
						}
					}
					else if (prev_point_catenary.z() > p_cat.z()){
						if ((prev_point_catenary.z() + approximation_) > _obstacles_near_catenary.z() && (p_cat.z() - approximation_) < _obstacles_near_catenary.z()){
							inside_z = true;
							obstacles_.z()=_obstacles_near_catenary.z();					
						}
					}
					else if (prev_point_catenary.z() == p_cat.z()){
						if ((prev_point_catenary.z() + approximation_) > _obstacles_near_catenary.z() && (prev_point_catenary.z() - approximation_) < _obstacles_near_catenary.z()){
							inside_z = true;
							obstacles_.z()=_obstacles_near_catenary.z();					
						}
					}
					else
						inside_z = false;

					if (inside_x && inside_y && inside_z){
						count++;
						// points_obs_close_cat _points2;
						_points2.cat =  p_cat;
						_points2.obs = _obstacles_near_catenary;
						// _v_p_cat_collision.push_back(p_cat);
						_v_obs_collision_cat.push_back(_points2);
					}
				}
				prev_point_catenary.x() = _points_catenary[i].x;
				prev_point_catenary.y() = _points_catenary[i].y;
				prev_point_catenary.z() = _points_catenary[i].z;
				first_point_cat = false;

				// Analysis II: To check if there are catenary points above z= 0.0
				if (p_cat.z() < bound_z_negative){
					count_negative_z++;
					if ( p_cat.z() < lower_point_cat_z)
						lower_point_cat_z = p_cat.z();
				}
			}

			points_obs_close_cat _v_av_close_oc;
			computeAveragePoint(_v_obs_close_cat,_v_av_close_oc);

			//Get distance between average points in collision from obstacles and catenary. 
			// double average_pos_obs_x, average_pos_obs_y, average_pos_obs_z;
			// average_pos_obs_x = average_pos_obs_y = average_pos_obs_z = 0.0;	
			// double average_p_cat_collision_x, average_p_cat_collision_y, average_p_cat_collision_z;
			// average_p_cat_collision_x = average_p_cat_collision_y = average_p_cat_collision_z = 0.0;	
			// for(int i = 0; i < _v_obs_collision_cat.size(); i++){
			// 	average_pos_obs_x = (_v_obs_collision_cat[i].x() + average_pos_obs_x)/(1.0 + i);
			// 	average_pos_obs_y = (_v_obs_collision_cat[i].y() + average_pos_obs_y)/(1.0 + i);
			// 	average_pos_obs_z = (_v_obs_collision_cat[i].z() + average_pos_obs_z)/(1.0 + i);
			// 	average_p_cat_collision_x = (_v_p_cat_collision[i].x() + average_p_cat_collision_x)/(1.0 + i);
			// 	average_p_cat_collision_y = (_v_p_cat_collision[i].y() + average_p_cat_collision_y)/(1.0 + i);
			// 	average_p_cat_collision_z = (_v_p_cat_collision[i].z() + average_p_cat_collision_z)/(1.0 + i);
			// }

			points_obs_close_cat _v_av_coll_oc;	
			computeAveragePoint(_v_obs_collision_cat, _v_av_coll_oc);
			double _d_av_cat_obs = (_v_av_coll_oc.cat - _v_av_coll_oc.obs).norm();

			// double n_collision = _v_obs_collision_cat.size();
			// if (n_collision < 1.0)
			// 	n_collision = 1.0;

			// Eigen::Vector3d _p_average_pos_obs , _p_average_cat_coll;
			// _p_average_pos_obs = Eigen::Vector3d(average_pos_obs_x,average_pos_obs_y,average_pos_obs_z);
			// _p_average_cat_coll = Eigen::Vector3d(average_p_cat_collision_x,average_p_cat_collision_y,average_p_cat_collision_z);



			// double _dis_average_collision = (_p_average_pos_obs - _p_average_cat_coll).norm();
			// double _dis_average_collision = (_p_average_pos_obs - pose->estimate()).norm();
			double _dis_average_collision = (_v_av_coll_oc.obs - pose->estimate()).norm();

			double _dist_Z = 0.0;
			// if (pose->estimate().z() > average_pos_obs_z && count > 0)
			// 	_dist_Z = pose->estimate().z() - average_pos_obs_z;
			if (pose->estimate().z() > _v_av_coll_oc.obs.z() && count > 0)
				_dist_Z = pose->estimate().z() - _v_av_coll_oc.obs.z();

			if (_dis_average_collision < 0.0001)
				_dis_average_collision = 0.0001;
			
			double move_down_vertex = 10000.0*exp(10.0*_dist_Z);

			// printf(PRINTF_REGULAR);
			// printf("count = [%i] _dist_Z =[%f] move_down_vertex=[%f] mode_obstacle=[%i] _p_average_pos_obs=[%f %f %f] _p_average_cat_coll=[%f %f %f]\n", 
			// count, _dist_Z, move_down_vertex, mode_obstacle,
			// _p_average_pos_obs.x(),_p_average_pos_obs.y(),_p_average_pos_obs.z(),_p_average_cat_coll.x(),_p_average_cat_coll.y(),_p_average_cat_coll.z());
			
			// Computed Error[0]: Collision catenary with obstacle
			if (mode_obstacle == 0){
				_error[0] = 1000.0*pow(count,4)*move_down_vertex; 
				// _error[0] = 100000.0*pow(count,4)*_dis_average_collision*(_length_catenary ); 
				// printf("Error[0] = %f for Catenary in vertex[%i] = [%f %f %f]\n", _error[0], pose->id(),pose->estimate().x(),pose->estimate().y(),pose->estimate().z());
			}
			else if(mode_obstacle == 1) {
				_error[0] = 10000.0*pow(count,4)/(_length_catenary); 
			}
			else if(mode_obstacle == 2) {
				// if (count > 0)
					_error[0] = 100.0*count/(_length_catenary); 
				// else
				// 	_error[0] = 0.0;
				// printf("entro modo 2: vertex[%i]  count=[%i]  _length_catenary=[%f]\n",pose->id(),count,_length_catenary);
			}
			else if(mode_obstacle == 3) {
				if (_v_obs_close_cat.size() > 0)
					_error[0] = (120.0* exp (_radius - _d_av_cat_obs))/_length_catenary;
				else 
					_error[0] = 0.0;
			}
			else if(mode_obstacle == 4) {
				_error[0] = 0.0;
			}
			// Computed Error[1]
			_error[1] = 1.0*(_length_catenary -  _d_curr); 
			// Computed Error[2]
			// _error[2] = 1000.0*pow(count_negative_z,4)*_length_catenary;
			_error[2] = 100000000.0*(double)(count_negative_z)*_length_catenary;
			
			if (_error[2] != 0.0 || _error[0] != 0.0 )
				printf("error[2] in vertex[%i]  _error=[%f %f %f]\n",pose->id(),_error[0],_error[1],_error[2]);
			
			// if ( count > 0){
			// 	printf("entro modo [%i]: vertex[%i]  count=[%i]  _length_catenary=[%f] _error[0] = [%f]  _error[1] = [%f]  _error[2] = [%f]\n",mode_obstacle,pose->id(),count,_length_catenary,_error[0],_error[1],_error[2]);
			// }
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

		inline void setRadius(const float& _s_r){_radius = _s_r;}

		inline void setInitialPosUGV(const geometry_msgs::Point& _p) {_initial_pos_ugv = _p;}

		// inline void setLengthCatenary(double _v){_length_catenary = _v;} // Set the catenary length related with distance between UGV and vertex

		inline void setModeBelowObstacles(int _b){mode_obstacle = _b;} 

		inline void setBoundForBisection(double _fa,double _fb){
			bound_bisection_a = _fa; 
			bound_bisection_b = _fb;
		}

		virtual void computeAveragePoint(std::vector<points_obs_close_cat> _vp, points_obs_close_cat &_struc){
			double xc,yc,zc,xo,yo,zo;
			xc=yc=zc=xo=yo=zo= 0.0;
			for(size_t i=0 ; i < _vp.size() ; i++){
				xc = (_vp[i].cat.x() + xc) / (1.0 + i);
				yc = (_vp[i].cat.y() + yc) / (1.0 + i);
				zc = (_vp[i].cat.z() + zc) / (1.0 + i);
				xo = (_vp[i].obs.x() + xo) / (1.0 + i);
				yo = (_vp[i].obs.y() + yo) / (1.0 + i);
				zo = (_vp[i].obs.z() + zo) / (1.0 + i);
			}
			_struc.cat = Eigen::Vector3d(xc,yc,zc);
			_struc.obs = Eigen::Vector3d(xo,yo,zo);
		}


	protected: 
};
}

#endif