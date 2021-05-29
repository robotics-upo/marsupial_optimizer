#ifndef CERES_CONSTRAINS_KINEMATICS_UGV_HPP
#define CERES_CONSTRAINS_KINEMATICS_UGV_HPP


#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"
#include <ros/ros.h>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

class KinematicsFunctorUGV 
{

public:
  	KinematicsFunctorUGV(double weight_factor, double angle_bound): wf_(weight_factor), ang_(angle_bound) {}

  	template <typename T>
  	bool operator()(const T* const statePos1, const T* const statePos2, const T* const statePos3, T* residual) const 
	{
		// Kinematics for ugv XY Axes
		T vector1_ugv[2] = {statePos2[1]-statePos1[1],statePos2[2]-statePos1[2]};
		T vector2_ugv[2] = {statePos3[1]-statePos2[1],statePos3[2]-statePos2[2]};
		T dot_product_ugv = (vector2_ugv[0] * vector1_ugv[0]) + (vector2_ugv[1] * vector1_ugv[1]);
		T norm_vector1_ugv = sqrt((vector1_ugv[0] * vector1_ugv[0]) + (vector1_ugv[1] * vector1_ugv[1]));
		T norm_vector2_ugv = sqrt((vector2_ugv[0] * vector2_ugv[0]) + (vector2_ugv[1] * vector2_ugv[1]));
		T angle_ugv;
		T bound1_ugv = T(-ang_);
		T bound2_ugv = T(ang_);

		T dist_12 = sqrt((statePos1[1]-statePos2[1])*(statePos1[1]-statePos2[1]) + 
						 (statePos1[2]-statePos2[2])*(statePos1[2]-statePos2[2]) + 
						 (statePos1[3]-statePos2[3])*(statePos1[3]-statePos2[3]) );
		
		T dist_13 = sqrt((statePos3[1]-statePos2[1])*(statePos3[1]-statePos2[1]) + 
						 (statePos3[2]-statePos2[2])*(statePos3[2]-statePos2[2]) + 
						 (statePos3[3]-statePos2[3])*(statePos3[3]-statePos2[3]) );


		if (dist_12 < 0.001 || dist_13 < 0.001)
			angle_ugv = T{0.0};
		else
			angle_ugv = acos(dot_product_ugv / (norm_vector1_ugv*norm_vector2_ugv));


		T value = sqrt(angle_ugv*angle_ugv);

		if ( (angle_ugv < bound1_ugv) || (angle_ugv > bound2_ugv) ) 
			residual[0] =  wf_ * exp(2.0* value );
		else
			residual[0] = T(0.0);

		

		return true;
	}

	double wf_, ang_;

private:

};


#endif