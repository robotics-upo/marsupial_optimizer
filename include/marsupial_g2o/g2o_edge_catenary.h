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

#include "g2o_vertex_catenary_factor.h"

#define PRINTF_GREEN "\x1B[32m"
#define PRINTF_BLUE "\x1B[34m"



namespace g2o {

class G2OCatenaryEdge : public BaseBinaryEdge<3, geometry_msgs::Point, VertexPointXYZ, VertexCatenaryFactor>
{
	public:
		G2OCatenaryEdge(ros::NodeHandlePtr nhP);
        
		double _length_catenary;
		// double _porcentage = 1.1;

		std::vector<geometry_msgs::Point> _points_catenary;
		visualization_msgs::MarkerArray _catenary_marker;
		ros::Publisher catenary_marker_pub_;

		bisectionCatenary bc;

		void setLengthCatenary(double _v);	// Set the catenary length related with distance between UGV and vertex
		void markerPoints(visualization_msgs::MarkerArray _marker, std::vector<geometry_msgs::Point> _vector, int _suffix, int _n_v);	// Print the catenary between UGV and vertex 
		void clearMarkers(visualization_msgs::MarkerArray _marker,auto _s);

		int _id_marker;		// Related with Marker frame.id
		int _n_vertices;	// Number of Vertices that no are fixed
		int _prev_size_marker = 0; // Save the previus size value of the catenary for vertex optimized To delete marker 
		

		NearNeighbor _nn;
		Eigen::Vector3d _obstacles_near_catenary;
		pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN;
		pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points;
		float _radius ;

		double _d, _x, _y, _z;

		bool _debug;

		void computeError()
		{
    		const VertexPointXYZ * pose = static_cast<const VertexPointXYZ*> (_vertices[0]);
    		const VertexCatenaryFactor * multiplicative_factor_catenary = static_cast<const VertexCatenaryFactor*> (_vertices[1]);

			// Calculate Catenary
			_x = (pose->estimate().x()-_measurement.x);
			_y = (pose->estimate().y()-_measurement.y);
			_z = (pose->estimate().z()-_measurement.z);
			_d = sqrt( _x * _x + _y * _y + _z * _z);

			// _length_catenary = _d * _porcentage;
			_length_catenary = _d * multiplicative_factor_catenary->estimate();
			bc.setNumberPointsCatenary(_length_catenary*10.0);
			bc.configBisection(_length_catenary,_measurement.x,_measurement.y,_measurement.z,pose->estimate().x(),pose->estimate().y(),pose->estimate().z());
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
			int count = 0;
			for (size_t i = 0 ; i < _points_catenary.size() ; i++){
				Eigen::Vector3d p_cat;
				p_cat.x() = _points_catenary[i].x;
				p_cat.y() = _points_catenary[i].y;
				p_cat.z() = _points_catenary[i].z;

				if (_nn.radiusNearestObstacleVertex(kdT_From_NN , p_cat, obstacles_Points, _radius)){
					count ++;
				}
			}
			// if (count > 0)	
				// ROS_WARN("CAUTION !! Catenary between UGV and Vertex[id=%i][pos= %f %f %f] is touching  obstacles in %i points, length=[%f].",pose->id(),pose->estimate().x(),pose->estimate().y(),pose->estimate().z(),count,_length_catenary);
			// else	
				// ROS_INFO_COND(_debug, PRINTF_BLUE "Catenary between UGV and Vertex[id=%i][pos= %f %f %f] is NOT touching  obstacles, length=[%f].",pose->id(),pose->estimate().x(),pose->estimate().y(),pose->estimate().z(),_length_catenary );
				


			//Calculate Error depending of numbers of catenary points touching obstacles - To decrease depending on the multiplicative factor 
			if (multiplicative_factor_catenary->estimate() < 1.0)
				_error[0] = 100000.0; 
			else
				_error[0] = 1.0*pow(count,2); 
			ROS_INFO_COND(_debug, PRINTF_GREEN "Error=[%f]   multiplicative_factor_catenary[%i]->estimate()[%f]",_error[0],multiplicative_factor_catenary->id(),multiplicative_factor_catenary->estimate());
			
		}

		virtual bool read(std::istream& is);

		virtual bool write(std::ostream& os) const;

		//Take the position value of the UGV
		virtual void setMeasurement(const geometry_msgs::Point& _p) {
            _measurement = _p;
		}
		
		virtual void numberVerticesNotFixed(const int& _n_v) {
            _n_vertices = _n_v;
		}

		inline void readKDTree(const pcl::KdTreeFLANN <pcl::PointXYZ> &kdT_, pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_){
			kdT_From_NN = kdT_;
			obstacles_Points = o_p_;
		}

		inline void setRadius(const float& _s_r){
			_radius = _s_r;
		}

	protected: 
};
}

#endif