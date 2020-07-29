#pragma once

#ifndef G2O_DISTANCE_VERTEX_EDGE_H
#define G2O_DISTANCE_VERTEX_EDGE_H


#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/config.h"
#include "g2o/core/base_binary_edge.h"

namespace g2o {

class G2ODistanceVertexEdge : public BaseBinaryEdge<1, double, VertexPointXYZ, VertexPointXYZ>
{
	public:
		G2ODistanceVertexEdge();

		// typedef MatrixX::MapType JacobianType;

		VertexPointXYZ * pose1;
		VertexPointXYZ * pose2;
		
		double factor_= 10.0;
		// double size_trajectory_;

		
		void computeError()
		{
			pose1 = static_cast<VertexPointXYZ *> (_vertices[0]);
			pose2 = static_cast<VertexPointXYZ *> (_vertices[1]);

			double _d = (pose2->estimate() - pose1->estimate()).norm();

			_error[0] = factor_ * (_d - _measurement);
		}

		virtual bool read(std::istream& is);
		virtual bool write(std::ostream& os) const;

		virtual void setMeasurement(const double& m) {
			_measurement = m;
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
		// std::vector<JacobianType, Eigen::aligned_allocator<JacobianType> > _jacobianOplus; ///< jacobians of the edge (w.r.t. oplus)
	};

}

#endif