#pragma once

#ifndef G2O_KINEMATICS_EDGE_H
#define G2O_KINEMATICS_EDGE_H


#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/config.h"
#include "g2o/core/base_multi_edge.h"


namespace g2o 
{

	class G2OKinematicsEdge : public BaseMultiEdge<3, double>
	{
	public:
		G2OKinematicsEdge();
		
		double factor_ = 2.0;

		void computeError()
		{
			const VertexPointXYZ* pose1 = static_cast<const VertexPointXYZ*>(_vertices[0]);
			const VertexPointXYZ* pose2 = static_cast<const VertexPointXYZ*>(_vertices[1]);
			const VertexPointXYZ* pose3 = static_cast<const VertexPointXYZ*>(_vertices[2]);

			double dist12_xy = sqrt(pow((pose2->estimate().x()-pose1->estimate().x()),2)+pow((pose2->estimate().y()-pose1->estimate().y()),2));
			double dist23_xy = sqrt(pow((pose3->estimate().x()-pose2->estimate().x()),2)+pow((pose3->estimate().y()-pose2->estimate().y()),2));
			double dist12_xz = sqrt(pow((pose2->estimate().x()-pose1->estimate().x()),2)+pow((pose2->estimate().z()-pose1->estimate().z()),2));
			double dist23_xz = sqrt(pow((pose3->estimate().x()-pose2->estimate().x()),2)+pow((pose3->estimate().z()-pose2->estimate().z()),2));


			double angle_x_12 =acos((pose2->estimate().x()-pose1->estimate().x())/ dist12_xy);
		    double angle_x_23 =acos((pose3->estimate().x()-pose2->estimate().x())/ dist23_xy);

		    double angle_y_12 =asin((pose2->estimate().y()-pose1->estimate().y())/ dist12_xy);
		    double angle_y_23 =asin((pose3->estimate().y()-pose2->estimate().y())/ dist23_xy);

   		    double angle_z_12 =asin((pose2->estimate().z()-pose1->estimate().z())/ dist12_xz);
		    double angle_z_23 =asin((pose3->estimate().z()-pose2->estimate().z())/ dist23_xz);


			if ((angle_x_12 - angle_x_23) < _measurement)
				_error[0] = factor_ * (angle_x_12 - angle_x_23) ;
			else
				_error[0] = 0.0;
			
			if	((angle_y_12 - angle_y_23) < _measurement)
				_error[1] = factor_ * (angle_y_12 - angle_y_23) ;
			else
				_error[1] = 0.0;

			if ((angle_z_12 - angle_z_23) < _measurement)
				_error[2] = factor_ * (angle_z_12 - angle_z_23) ;
			else
				_error[2] = 0.0;

		}


		virtual bool read(std::istream& is);
		virtual bool write(std::ostream& os) const;

		virtual void setMeasurement(const double& m) {
            _measurement = m;
		}
	};
} 

#endif