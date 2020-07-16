#pragma once

#ifndef G2O_OBSTACLES_EDGE_H
#define G2O_OBSTACLES_EDGE_H


#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/config.h"
#include "g2o/core/base_unary_edge.h"
#include <vector>

// #include <pcl/search/kdtree.h>

#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include "marsupial_g2o/nearNeighbor.hpp"


namespace g2o {

class G2OObstaclesEdge : public BaseUnaryEdge<1, double, VertexPointXYZ>
{
	public:
		G2OObstaclesEdge();
        
		double size_trajectory_;

		int count = 1;
		int _id;

		NearNeighbor nn;
		Eigen::Vector3d near_;
		pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN;
		pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points;

		void computeError()
		{
    		const VertexPointXYZ * xyz = static_cast<const VertexPointXYZ*> (_vertices[0]);

			near_ = nn.nearestObstacleVertex(kdT_From_NN , xyz->estimate(),obstacles_Points);
			double _d = (xyz->estimate()-near_).norm();
			
			if (_d < _measurement)
				_error[0] = exp(_measurement - 2.0*_d) ; 
			else 
				_error[0] = 0.0;
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

		// Commented 09-07-2020.
		// virtual void linearizeOplus()
		// {
		// 	VertexPointXYZ * xyz = static_cast<VertexPointXYZ*> (_vertices[0]);
		// 	near_ = nn.nearestObstacleVertex(kdT_From_NN , xyz->estimate(),obstacles_Points);
		// 	float _d = (xyz->estimate()-near_).norm();
		// 	if (_d < 0.0000001)
		// 		_d= 0.0000001;

		// 	 _jacobianOplusXi(0,0) = -exp(_measurement - _d) * (xyz->estimate().x()-near_.x())/(_d) ;  
		// 	 _jacobianOplusXi(0,1) = -exp(_measurement - _d) * (xyz->estimate().y()-near_.y())/(_d) ;
		// 	 _jacobianOplusXi(0,2) = -exp(_measurement - _d) * (xyz->estimate().z()-near_.z())/(_d) ;
			// printf("[%i] xyz.x = %f  xyz.y = %f  xyz.z = %f near.x = %f near.y = %f  near.z = %f \n",
			// 		xyz->id(),xyz->estimate().x(),xyz->estimate().y(),xyz->estimate().z(),near_.x(),near_.y(),near_.z());
			// printf("J1 = %.8f   J2 = %.8f  J3 = %.8f \n", _jacobianOplusXi(0,0),_jacobianOplusXi(0,1),_jacobianOplusXi(0,2));
			// printf("_d = %.8f   _m = %f\n",_d,_measurement);
		// }

	protected:

};
}

#endif