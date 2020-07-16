#pragma once

#ifndef G2O_DISTANCE_XYZ_EDGE_H
#define G2O_DISTANCE_XYZ_EDGE_H


#include "g2o_vertex_pointxyz.h"
#include "g2o/config.h"
#include "g2o/core/base_binary_edge.h"

namespace g2o {

class G2ODistanceXYZEdge : public BaseBinaryEdge<1, std::vector<float>, VertexXYZ, VertexXYZ>
{
	public:
		G2ODistanceXYZEdge();
		VertexXYZ * xyz1;
		VertexXYZ * xyz2;


		void computeError()
		{
			xyz1 = static_cast<VertexXYZ *> (_vertices[0]);
			xyz2 = static_cast<VertexXYZ *> (_vertices[1]);
			
			// _error[0] = (xyz2->estimate().x() - xyz1->estimate().x()) - _measurement[0];

			_error[0] = ((xyz2->estimate().x() - xyz1->estimate().x()) - _measurement[0]) + 
						((xyz2->estimate().y() - xyz1->estimate().y()) - _measurement[1]) +
						((xyz2->estimate().z() - xyz1->estimate().z()) - _measurement[2]) ;
			
		}

		virtual bool read(std::istream& is);
		virtual bool write(std::ostream& os) const;

		virtual void setMeasurement(const std::vector<float>& m) {
			_measurement = m;
		}

		virtual void linearizeOplus()
		{
			// _jacobianOplusXi=-Matrix3::Identity();
    		// _jacobianOplusXj= Matrix3::Identity();
			_jacobianOplusXi(0,0) = -1.0;  
			_jacobianOplusXi(0,1) = -1.0;   
			_jacobianOplusXi(0,2) = -1.0;   
			_jacobianOplusXj(0,0) = 1.0;  
			_jacobianOplusXj(0,1) = 1.0;   
			_jacobianOplusXj(0,2) = 1.0;  
		}


	};

}

#endif