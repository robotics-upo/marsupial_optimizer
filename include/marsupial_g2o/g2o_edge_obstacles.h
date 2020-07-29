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
		double factor_ = 2.0;

		NearNeighbor nn;
		Eigen::Vector3d near_;
		pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN;
		pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points;

		void computeError()
		{
    		const VertexPointXYZ * pose = static_cast<const VertexPointXYZ*> (_vertices[0]);

			near_ = nn.nearestObstacleVertex(kdT_From_NN , pose->estimate(),obstacles_Points);
			double _d = (pose->estimate()-near_).norm();
			
			if (_d < _measurement){
				_error[0] = factor_ * exp(_measurement - 2.0*_measurement*_d) ; 
				// _error[0] = factor_* (_measurement-_d)/_measurement; 
				// printf("error[%i] = [%f]\n",pose->id(),_error[0]);
				// _error[0] = 1000.0*exp(_measurement - 2.0*_d) ; 
			}
			else {
				_error[0] = 0.0;
				// printf("error[%i] = [%f]\n",pose->id(),_error[0]);
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

		// virtual void linearizeOplus()
		// {
		// 	VertexPointXYZ * pose = static_cast<VertexPointXYZ*> (_vertices[0]);
		// 	near_ = nn.nearestObstacleVertex(kdT_From_NN , pose->estimate(),obstacles_Points);
		// 	float _d = (pose->estimate()-near_).norm();
		// 	if (_d < 0.0000001)
		// 		_d= 0.0000001;

		// 	 _jacobianOplusXi(0,0) = factor_* (near_.x()-pose->estimate().x())/(_measurement * _d);  
		// 	 _jacobianOplusXi(0,1) = factor_* (near_.y()-pose->estimate().y())/(_measurement * _d);
		// 	 _jacobianOplusXi(0,2) = factor_* (near_.z()-pose->estimate().z())/(_measurement * _d);
		// 	// printf("_d = %.8f   _m = %f\n",_d,_measurement);
		// }

	protected:

};
}

#endif