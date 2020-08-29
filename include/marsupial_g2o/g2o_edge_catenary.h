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



namespace g2o {


class G2OCatenaryEdge : public BaseBinaryEdge<3, vector<double>, VertexPointXYZ, VertexCatenaryLength>
{
	public:
		G2OCatenaryEdge(ros::NodeHandlePtr nhP);
        
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
		int n_cat_computed = 0;

		NearNeighbor _nn;
		Eigen::Vector3d _obstacles_near_catenary;
		Eigen::Vector3d obstacles_;
		Eigen::Vector3d prev_point_catenary;
		pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN;
		pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points;
		float _radius ;

		double _d_curr, _x, _y, _z;
		bool _debug;
		double approximation_;
		double _length_cat_;

		geometry_msgs::Point _initial_pos_ugv;
	

		std::vector<Eigen::Vector3d> _v_obs_collision_cat;
		std::vector<Eigen::Vector3d> _v_p_cat_collision;

		void computeError()
		{
			// printf("========================Start computeError() ======================\n");
			
    		const VertexPointXYZ * pose = static_cast<const VertexPointXYZ*> (_vertices[0]);
    		const VertexCatenaryLength * l_catenary = static_cast<const VertexCatenaryLength*> (_vertices[1]);

			if (n_cat_computed < pose->id())
				n_cat_computed = pose->id();

			// Calculate Catenary
			_x = (pose->estimate().x()-_initial_pos_ugv.x);
			_y = (pose->estimate().y()-_initial_pos_ugv.y);
			_z = (pose->estimate().z()-_initial_pos_ugv.z);
			_d_curr = sqrt( _x * _x + _y * _y + _z * _z); // Distance UGV and Vertex

			_length_catenary = l_catenary->estimate();

			if (_length_catenary > _d_curr){
				bc.setNumberPointsCatenary(_length_catenary*10.0);
				bc.configBisection(_length_catenary,_initial_pos_ugv.x,_initial_pos_ugv.y,_initial_pos_ugv.z,pose->estimate().x(),pose->estimate().y(),pose->estimate().z());
				// _length_cat_ = _length_catenary;
			}
			else
			{
				bc.setNumberPointsCatenary(_d_curr*10.0);
				bc.configBisection(_d_curr*1.001,_initial_pos_ugv.x,_initial_pos_ugv.y,_initial_pos_ugv.z,pose->estimate().x(),pose->estimate().y(),pose->estimate().z());
				// _length_catenary = _d_curr*1.001; 
			}
			_points_catenary.clear();
			bc.getPointCatenary3D(_points_catenary);
			
			// Print Catenary Marker between each vertex and UGV
			if (_prev_size_marker >= _points_catenary.size() )
				clearMarkers(_catenary_marker,_prev_size_marker);
			else
				clearMarkers(_catenary_marker,_points_catenary.size());
			_id_marker = pose->id();
			markerPoints(_catenary_marker ,_points_catenary,_id_marker,_n_vertices);
			_prev_size_marker = _points_catenary.size();

			// To Obtain KdTree for each point in every calculated catenary 
			int count = 0 ; 
			int count_negative_z = 0;
			bool first_point_cat = true;
			bool inside_x, inside_y, inside_z;
			_v_obs_collision_cat.clear();
			_v_p_cat_collision.clear();

			for (size_t i = 0 ; i < _points_catenary.size() ; i++){
				Eigen::Vector3d p_cat;
				p_cat.x() = _points_catenary[i].x;
				p_cat.y() = _points_catenary[i].y;
				p_cat.z() = _points_catenary[i].z;
				
				_obstacles_near_catenary = _nn.nearestObstacleVertex(kdT_From_NN , p_cat,obstacles_Points);
				double _d_cat_obs = (p_cat-_obstacles_near_catenary).norm();
				
				double _d_points_cat = (p_cat-prev_point_catenary).norm();
				approximation_ = _d_points_cat;

				inside_x = inside_y = inside_z = false;

				// To check if there is obstacle between two catenary points
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
						_v_obs_collision_cat.push_back(_obstacles_near_catenary);
						_v_p_cat_collision.push_back(p_cat);
					}
				}
				prev_point_catenary.x() = _points_catenary[i].x;
				prev_point_catenary.y() = _points_catenary[i].y;
				prev_point_catenary.z() = _points_catenary[i].z;
				first_point_cat = false;

				// To check if there are catenary points above z= 0.0
				if (p_cat.z() = _points_catenary[i].z < 0.001){
					count_negative_z++;
				}
			}


			//Get distance between average points in collision from obstacles and catenary. 
			double average_pos_obs_x, average_pos_obs_y, average_pos_obs_z;
			average_pos_obs_x = average_pos_obs_y = average_pos_obs_z = 0.0;	
			double average_p_cat_collision_x, average_p_cat_collision_y, average_p_cat_collision_z;
			average_p_cat_collision_x = average_p_cat_collision_y = average_p_cat_collision_z = 0.0;	
			for(int i = 0; i < _v_obs_collision_cat.size(); i++){
				average_pos_obs_x = _v_obs_collision_cat[i].x() + average_pos_obs_x;
				average_pos_obs_y = _v_obs_collision_cat[i].y() + average_pos_obs_y;
				average_pos_obs_z = _v_obs_collision_cat[i].z() + average_pos_obs_z;
				average_p_cat_collision_x = _v_p_cat_collision[i].x() + average_p_cat_collision_x;
				average_p_cat_collision_y = _v_p_cat_collision[i].x() + average_p_cat_collision_y;
				average_p_cat_collision_z = _v_p_cat_collision[i].x() + average_p_cat_collision_z;
			}
			Eigen::Vector3d _p_average_pos_obs , _p_average_cat_coll;
			_p_average_pos_obs.x() = average_pos_obs_x;
			_p_average_pos_obs.y() = average_pos_obs_y;
			_p_average_pos_obs.z() = average_pos_obs_z;
			_p_average_cat_coll.x() = average_p_cat_collision_x;
			_p_average_cat_coll.y() = average_p_cat_collision_z;
			_p_average_cat_coll.z() = average_p_cat_collision_y;

			// double _dis_average_collision = (_p_average_pos_obs - _p_average_cat_coll).norm();
			double _dis_average_collision = (_p_average_pos_obs - pose->estimate()).norm();

			if (_dis_average_collision < 0.0001)
				_dis_average_collision = 0.0001;
			

			// Computed Error[0]: Collision catenary with obstacle
			_error[0] = 100000.0*pow(count,4)/(_length_catenary *_dis_average_collision); 
			// _error[0] = 100000.0*pow(count,4)*_dis_average_collision*(_length_catenary ); 
			// Computed Error[1]
			_error[1] = 8.0*(_length_catenary -  _d_curr); 
			// Computed Error[2]
			_error[2] = 100000.0*pow(count_negative_z,4)*_length_catenary;

		}

		virtual bool read(std::istream& is);

		virtual bool write(std::ostream& os) const;

		//Take the position value of the UGV
		virtual void setMeasurement(const vector<double> _p) {
			_measurement = _p;
		}
		
		virtual void numberVerticesNotFixed(const int& _n_v) {_n_vertices = _n_v;}

		inline void readKDTree(const pcl::KdTreeFLANN <pcl::PointXYZ> &kdT_, pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_){
			kdT_From_NN = kdT_;
			obstacles_Points = o_p_;
		}

		inline void setRadius(const float& _s_r){_radius = _s_r;}

		inline void setInitialPosUGV(const geometry_msgs::Point& _p) {_initial_pos_ugv = _p;}

		inline void setLengthCatenary(double _v){_length_catenary = _v;} // Set the catenary length related with distance between UGV and vertex


		// void initializeVector(void){
		// 	// printf("initializeVector\n");
		// 	for(int i=0; i< 20; i++){
		// 		_v_count.push_back(0);
		// 		_v_count_negative_z.push_back(0);
		// 		_v_length_cat.push_back(0.0);
		// 	}
		// }


	protected: 
};
}

#endif