#pragma once

#ifndef G2O_TIME_EDGE_H
#define G2O_TIME_EDGE_H


#include "g2o_vertex_timediff.h"
#include "g2o/config.h"
#include "g2o/core/base_unary_edge.h"

#include "misc.hpp"

namespace g2o {

class G2OTimeEdge : public BaseUnaryEdge<1, double, VertexTimeDiff>
{
	public:
		G2OTimeEdge();

		VertexTimeDiff* VTime;

		void computeError()
		{
			VTime = static_cast<VertexTimeDiff *> (_vertices[0]);
			_error[0] = VTime->estimate() - _measurement;

		}

		virtual bool read(std::istream& is);
		virtual bool write(std::ostream& os) const;

		virtual void setMeasurement(const double& m) {
			_measurement = m;
		}

		void linearizeOplus()
		{
		  _jacobianOplusXi( 0 , 0 ) = 1.0;
		}

	protected:
		// std::vector<JacobianType, Eigen::aligned_allocator<JacobianType> > _jacobianOplus; ///< jacobians of the edge (w.r.t. oplus)
	};

}

#endif