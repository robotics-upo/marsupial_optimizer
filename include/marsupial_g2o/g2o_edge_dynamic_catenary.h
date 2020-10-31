#pragma once

#ifndef G2O_DYNAMIC_CATENARY_EDGE_H
#define G2O_DYNAMIC_CATENARY_EDGE_H

#include "g2o_vertex_timediff.h"
#include "g2o/config.h"
#include "g2o/core/base_multi_edge.h"
#include "g2o_vertex_catenary_length.h"

namespace g2o 
{

	class G2ODynamicCatenaryEdge : public BaseMultiEdge<1, double>
	{
	public:
		G2ODynamicCatenaryEdge();

		const VertexCatenaryLength *l_catenary1; 
		const VertexCatenaryLength *l_catenary2; 
		const VertexTimeDiff* deltaT1;
		const VertexTimeDiff* deltaT2;

		double factor_ = 10.0;
		
		void computeError()
		{
			l_catenary1 = static_cast<const VertexCatenaryLength*> (_vertices[0]);
			l_catenary2 = static_cast<const VertexCatenaryLength*> (_vertices[1]);
			deltaT1 = static_cast<const VertexTimeDiff*>(_vertices[2]);
			deltaT2 = static_cast<const VertexTimeDiff*>(_vertices[3]);


			double _dyn = (l_catenary2 - l_catenary1)/(deltaT2 - deltaT1);

			if (fabs(_dyn) > _measurement) 
				_error[0] = factor_ * (_dyn);
			else
				_error[0] = 0.0;
		}

		virtual void setMeasurement(const double& m) {
			_measurement = m;
		}

		virtual bool read(std::istream& is);
		virtual bool write(std::ostream& os) const;
		
	};
} 

#endif