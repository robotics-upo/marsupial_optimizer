#pragma once

#ifndef G2O_DISTANCE_TRAJECTORY_EDGE_H
#define G2O_DISTANCE_TRAJECTORY_EDGE_H


#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/config.h"
// #include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_multi_edge.h"

namespace g2o {

// class G2ODistanceTrajectoryEdge : public BaseBinaryEdge<1, double, VertexPointXYZ, VertexPointXYZ>
class G2ODistanceTrajectoryEdge : public BaseMultiEdge<1, double>
{
	public:
		G2ODistanceTrajectoryEdge();

		double size_trajectory_;
		double dT_ = 0;


		void computeError()
		{
			for(unsigned int i=0; i<size_trajectory_; i++)
			{
				printf("here 1 !!!\n");
				double d_;
				VertexPointXYZ * xyz1 = static_cast<VertexPointXYZ *> (_vertices[i]);
				VertexPointXYZ * xyz2 = static_cast<VertexPointXYZ *> (_vertices[1+i]);

				d_ = pow(xyz1->estimate().x()-xyz2->estimate().x(),2)+pow(xyz1->estimate().y()-xyz2->estimate().y(),2)+pow(xyz1->estimate().z()-xyz2->estimate().z(),2);
				dT_ = dT_ + d_;
				printf("xyz1: x=%f y=%f z=%f  xyz2: x=%f y=%f z=%f  d_=%d  dT_=%d\n", 
				xyz1->estimate().x(),xyz1->estimate().y(),xyz1->estimate().z(),xyz2->estimate().x(),xyz2->estimate().y(),xyz2->estimate().z() ,d_,dT_);
				printf("here 2 !!!\n");


			}
			_error[0] = dT_ - _measurement;
	
			
		}

		virtual bool read(std::istream& is);
		virtual bool write(std::ostream& os) const;

		virtual void setMeasurement(const double& m) {
			_measurement = m;
		}

		inline void setSizeTrajectory(double v_){size_trajectory_ = v_;} 


	};

}

#endif