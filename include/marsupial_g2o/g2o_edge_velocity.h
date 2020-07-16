#pragma once

#ifndef G2O_VELOCITY_EDGE_H
#define G2O_VELOCITY_EDGE_H


#include "g2o_vertex_timediff.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/config.h"
#include "g2o/core/base_multi_edge.h"

#include "misc.hpp"

namespace g2o 
{

	class G2OVelocityEdge : public BaseMultiEdge<1, double>
	{
	public:
		G2OVelocityEdge();

		const VertexPointXYZ* pose1;
		const VertexPointXYZ* pose2;
		const VertexTimeDiff* deltaT;

		void computeError()
		{
			pose1 = static_cast<const VertexPointXYZ*>(_vertices[0]);
			pose2 = static_cast<const VertexPointXYZ*>(_vertices[1]);
			deltaT = static_cast<const VertexTimeDiff*>(_vertices[2]);


			double _d = (pose2->estimate() - pose1->estimate()).norm();
			double _v =  _d/ deltaT->estimate();

			if ((_v > _measurement + _measurement/5.0) || (_v < _measurement - _measurement/5.0))
				_error[0] = (_v - _measurement);
			else
				_error[0] = 0.0;
		}

		virtual void setMeasurement(const double& m) {
			_measurement = m;
		}

		virtual bool read(std::istream& is);
		virtual bool write(std::ostream& os) const;
		
		// virtual void linearizeOplus()
		// {
		// 	pose1 = static_cast<VertexPointXYZ *> (_vertices[0]);
		// 	pose2 = static_cast<VertexPointXYZ *> (_vertices[1]);
		// 	deltaT = static_cast<const VertexTimeDiff*>(_vertices[2]);
			
		// 	float _d = (pose2->estimate()-pose1->estimate()).norm();
		// 	if (_d < 0.0000001)
		// 		_d= 0.0000001;

		// 	_jacobianOplusXi(0,0) = -(pose2->estimate().x()-pose1->estimate().x())/(_d);  
		// 	_jacobianOplusXi(0,1) = -(pose2->estimate().y()-pose1->estimate().y())/(_d);   
		// 	_jacobianOplusXi(0,2) = -(pose2->estimate().z()-pose1->estimate().z())/(_d);   
		// 	_jacobianOplusXj(0,0) = (pose2->estimate().x()-pose1->estimate().x())/(_d);  
		// 	_jacobianOplusXj(0,1) = (pose2->estimate().y()-pose1->estimate().y())/(_d);   
		// 	_jacobianOplusXj(0,2) = (pose2->estimate().z()-pose1->estimate().z())/(_d);  
		// }

	};
} 

#endif