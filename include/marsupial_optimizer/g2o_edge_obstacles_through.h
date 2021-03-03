#pragma once

#ifndef G2O_OBSTACLES_THROUGH_EDGE_H
#define G2O_OBSTACLES_THROUGH_EDGE_H


#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/config.h"
#include "g2o/core/base_binary_edge.h"
#include <vector>

// #include <pcl/search/kdtree.h>

#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include "marsupial_optimizer/near_neighbor.hpp"


namespace g2o {

class G2OThroughObstaclesEdge : public BaseBinaryEdge<1, double, VertexPointXYZ,VertexPointXYZ>
{
	public:
		G2OThroughObstaclesEdge();
        
		double size_trajectory_;

		int num_points = 4;
		std::vector<Eigen::Vector3d> _vector_points_line;
		std::vector<Eigen::Vector3d> _vectorMainObstacles;
		Eigen::Vector3d _main_obstacle, _main_obstacle_marker;
		double _sum_obstacle_x, _sum_obstacle_y, _sum_obstacle_z;
		int _size_vector;
		int _size_vector_marker = 0;

		double approximation_ = 0.2;
		double factor_ = 100000.0;

		double ws_limit = 100.0;

		NearNeighbor nn;
		Eigen::Vector3d near_;
		pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN;
		pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points;

		void computeError()
		{
    		const VertexPointXYZ * pose1 = static_cast<const VertexPointXYZ*> (_vertices[0]);
    		const VertexPointXYZ * pose2 = static_cast<const VertexPointXYZ*> (_vertices[1]);

			bool inside_x, inside_y, inside_z;

			if ((pose1->estimate().x() >  ws_limit || pose1->estimate().x() <  -1.0*ws_limit || pose1->estimate().y() >  ws_limit || pose1->estimate().y() <  -1.0*ws_limit || pose1->estimate().z() >  ws_limit || pose1->estimate().z() <  0.0*ws_limit ) ||
				(pose2->estimate().x() >  ws_limit || pose2->estimate().x() <  -1.0*ws_limit || pose2->estimate().y() >  ws_limit || pose2->estimate().y() <  -1.0*ws_limit || pose2->estimate().z() >  ws_limit || pose2->estimate().z() <  0.0*ws_limit )){
					_error[0] = factor_ *(pose1->estimate()-pose2->estimate()).norm();
			}
			else{
				_vector_points_line.clear();
				_vectorMainObstacles.clear();

				straightTrajectoryVertices(pose1->estimate().x(),pose1->estimate().y(),pose1->estimate().z(),pose2->estimate().x(),pose2->estimate().y(),pose2->estimate().z(),num_points,_vector_points_line);
				
				for (size_t i =0; i <_vector_points_line.size();i++){

					inside_x = false;
					inside_y = false;
					inside_z = false;
					Eigen::Vector3d obstacles_;
					near_ = nn.nearestObstacleVertex(kdT_From_NN , _vector_points_line[i],obstacles_Points);

					if (pose1->estimate().x() < pose2->estimate().x()){
						if ((pose1->estimate().x() - approximation_) < near_.x() && (pose2->estimate().x() + approximation_) > near_.x()){
							inside_x = true;
							obstacles_.x()=near_.x();					
						}
					}
					else if (pose1->estimate().x() > pose2->estimate().x()){
						if ((pose1->estimate().x() + approximation_) > near_.x() && (pose2->estimate().x() - approximation_) < near_.x()){
							inside_x = true;
							obstacles_.x()=near_.x();					
						}
					}
					else if (pose1->estimate().x() == pose2->estimate().x()){
						if ((pose1->estimate().x() + approximation_) > near_.x() && (pose1->estimate().x() - approximation_) < near_.x()){
							inside_x = true;
							obstacles_.x()=near_.x();					
						}
					}
					else
						inside_x = false;


					if (pose1->estimate().y() < pose2->estimate().y()){
						if ((pose1->estimate().y() - approximation_) < near_.y() && (pose2->estimate().y() + approximation_) > near_.y()){
							inside_y = true;
							obstacles_.y()=near_.y();					
						}
					}
					else if (pose1->estimate().y() > pose2->estimate().y()){
						if ((pose1->estimate().y() + approximation_) > near_.y() &&  (pose2->estimate().y() - approximation_) < near_.y()){
							inside_y = true;
							obstacles_.y()=near_.y();					
						}
					}
					else if (pose1->estimate().y() == pose2->estimate().y()){
						if ((pose1->estimate().y() + approximation_) > near_.y() && (pose1->estimate().y() - approximation_) < near_.y()){
							inside_y = true;
							obstacles_.y()=near_.y();					
						}
					}
					else
						inside_y = false;

					
					if (pose1->estimate().z() < pose2->estimate().z()){
						if ((pose1->estimate().z() - approximation_) < near_.z() && (pose2->estimate().z() + approximation_) > near_.z()){
							inside_z = true;
							obstacles_.z()=near_.z();					
						}
					}
					else if (pose1->estimate().z() > pose2->estimate().z()){
						if ((pose1->estimate().z() + approximation_) > near_.z() && (pose2->estimate().z() - approximation_) < near_.z()){
							inside_z = true;
							obstacles_.z()=near_.z();					
						}
					}
					else if (pose1->estimate().z() == pose2->estimate().z()){
						if ((pose1->estimate().z() + approximation_) > near_.z() && (pose1->estimate().z() - approximation_) < near_.z()){
							inside_z = true;
							obstacles_.z()=near_.z();					
						}
					}
					else
						inside_z = false;

					if (inside_x && inside_y && inside_z){
						_vectorMainObstacles.push_back(obstacles_);
					}
				}

				_sum_obstacle_x = _sum_obstacle_y = _sum_obstacle_z = 0.0;
				if (_vectorMainObstacles.size() == 0)
					_size_vector = 0;
				else
					_size_vector = _vectorMainObstacles.size();

				for (int k=0 ; k < _size_vector; k++){
					_sum_obstacle_x = _vectorMainObstacles[k].x() + _sum_obstacle_x;
					_sum_obstacle_y = _vectorMainObstacles[k].y() + _sum_obstacle_y;
					_sum_obstacle_z = _vectorMainObstacles[k].z() + _sum_obstacle_z;
				}
				_main_obstacle.x() = (_sum_obstacle_x) / _size_vector;
				_main_obstacle.y() = (_sum_obstacle_y) / _size_vector;
				_main_obstacle.z() = (_sum_obstacle_z) / _size_vector;


				double _do1 = (pose1->estimate()-_main_obstacle).norm();
				double _do2 = (pose2->estimate()-_main_obstacle).norm();

				if (_size_vector >= _size_vector_marker){
					_main_obstacle_marker = _main_obstacle;
					_size_vector_marker = _size_vector;	
				}

				if (inside_x && inside_y && inside_z)
				{
					_error[0] = factor_ *(_do1+_do2); 

				}
				else
					_error[0] = 0.0; 
				}
		}

		virtual bool read(std::istream& is);
		virtual bool write(std::ostream& os) const;

		virtual void setMeasurement(const double& m) {
            _measurement = m;
		}

		inline void readKDTree(const pcl::KdTreeFLANN <pcl::PointXYZ> &kdT_, pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_)
		{
			kdT_From_NN = kdT_;
			obstacles_Points = o_p_;
		}

		inline void setSizeTrajectory(double v_){size_trajectory_ = v_;} 

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


		// virtual void linearizeOplus()
		// {
		// 	pose1 = static_cast<VertexPointXYZ *> (_vertices[0]);
		// 	pose2 = static_cast<VertexPointXYZ *> (_vertices[1]);
			
		// 	float _d = (pose2->estimate()-pose1->estimate()).norm();
		// 	if (_d < 0.0000001)
		// 		_d= 0.0000001;

		// 	_jacobianOplusXi(0,0) = -1.0*(pose2->estimate().x()-pose1->estimate().x()) / (_d);  
		// 	_jacobianOplusXi(0,1) = -1.0*(pose2->estimate().y()-pose1->estimate().y()) / (_d);   
		// 	_jacobianOplusXi(0,2) = -1.0*(pose2->estimate().z()-pose1->estimate().z()) / (_d);   
		// 	_jacobianOplusXj(0,0) =  1.0*(pose2->estimate().x()-pose1->estimate().x()) / (_d);  
		// 	_jacobianOplusXj(0,1) =  1.0*(pose2->estimate().y()-pose1->estimate().y()) / (_d);   
		// 	_jacobianOplusXj(0,2) =  1.0*(pose2->estimate().z()-pose1->estimate().z()) / (_d);  
		// }

	protected:

};
}

#endif