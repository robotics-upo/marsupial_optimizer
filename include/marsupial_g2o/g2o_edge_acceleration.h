#pragma once

#ifndef G2O_ACCELERATION_EDGE_H
#define G2O_ACCELERATION_EDGE_H


#include "g2o_vertex_timediff.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/config.h"
#include "g2o/core/base_multi_edge.h"

#include "misc.hpp"

namespace g2o 
{

	class G2OAccelerationEdge : public BaseMultiEdge<1, double>
	{
	public:
		G2OAccelerationEdge();

		const VertexPointXYZ* pose1;
		const VertexPointXYZ* pose2;
		const VertexPointXYZ* pose3;
		const VertexTimeDiff* deltaT1;
		const VertexTimeDiff* deltaT2;


		void computeError()
		{
			pose1 = static_cast<const VertexPointXYZ*>(_vertices[0]);
			pose2 = static_cast<const VertexPointXYZ*>(_vertices[1]);
			pose3 = static_cast<const VertexPointXYZ*>(_vertices[2]);
			deltaT1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
			deltaT2 = static_cast<const VertexTimeDiff*>(_vertices[4]);


			double _d1 = (pose2->estimate() - pose1->estimate()).norm();
			double _d2 = (pose3->estimate() - pose2->estimate()).norm();
			double _v1 =  _d1/ deltaT1->estimate();
			double _v2 =  _d2/ deltaT2->estimate();
			double _a = (_v2-_v1)/(deltaT1->estimate()+deltaT2->estimate());

			if (_a > _measurement ) 
				_error[0] = (_a - _measurement);
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