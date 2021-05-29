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
  	bool operator()(const T* const statePos1, const T* const statePos2, const T* const statePos3, const T* const statePos4, T* residual) const 
	{
		// Kinematics for ugv XY Axes
		T vector1[3] = {statePos2[1]-statePos1[1],statePos2[2]-statePos1[2],statePos2[3]-statePos1[3]};
		T vector2[3] = {statePos3[1]-statePos2[1],statePos3[2]-statePos2[2],statePos3[3]-statePos2[3]};
		T vector3[3] = {statePos4[1]-statePos3[1],statePos4[2]-statePos3[2],statePos4[3]-statePos3[3]};
		T dot_product1 = (vector2[0] * vector1[0]) + (vector2[1] * vector1[1]) + (vector2[2] * vector1[2]);
		T dot_product2 = (vector3[0] * vector2[0]) + (vector3[1] * vector2[1]) + (vector3[2] * vector2[2]);
		T norm_vector1 = sqrt((vector1[0] * vector1[0]) + (vector1[1] * vector1[1]) + (vector1[2] * vector1[2]));
		T norm_vector2 = sqrt((vector2[0] * vector2[0]) + (vector2[1] * vector2[1]) + (vector2[2] * vector2[2]));
		T norm_vector3 = sqrt((vector3[0] * vector3[0]) + (vector3[1] * vector3[1]) + (vector3[2] * vector3[2]));
		T angle_ugv1, angle_ugv2;
		T bound1_ugv = T(-ang_);
		T bound2_ugv = T(ang_);

		T dist_12 = sqrt((statePos1[1]-statePos2[1])*(statePos1[1]-statePos2[1]) + 
						 (statePos1[2]-statePos2[2])*(statePos1[2]-statePos2[2]) + 
						 (statePos1[3]-statePos2[3])*(statePos1[3]-statePos2[3]) );
		
		T dist_23 = sqrt((statePos3[1]-statePos2[1])*(statePos3[1]-statePos2[1]) + 
						 (statePos3[2]-statePos2[2])*(statePos3[2]-statePos2[2]) + 
						 (statePos3[3]-statePos2[3])*(statePos3[3]-statePos2[3]) );
		
		T dist_34 = sqrt((statePos4[1]-statePos3[1])*(statePos4[1]-statePos3[1]) + 
						 (statePos4[2]-statePos3[2])*(statePos4[2]-statePos3[2]) + 
						 (statePos4[3]-statePos3[3])*(statePos4[3]-statePos3[3]) );

		//Get first Angle
		if (dist_12 < 0.001 || dist_23 < 0.001)
			angle_ugv1 = T{0.0};
		else
			angle_ugv1 = acos(dot_product1 / (norm_vector1*norm_vector2));

		//Get second Angle
		if (dist_23 < 0.001 || dist_34 < 0.001)
			angle_ugv2 = T{0.0};
		else
			angle_ugv2 = acos(dot_product2 / (norm_vector2*norm_vector3));

		T diff_angle_ = (angle_ugv1-angle_ugv2);
		T value_xy1 = sqrt(diff_angle_*diff_angle_);
		T value_xy2 = sqrt(angle_ugv1*angle_ugv1);

		
		if ( (diff_angle_ < bound1_ugv) || (diff_angle_ > bound2_ugv) ) 
			residual[0] =  wf_ * exp(2.0* value_xy1 );
		else
			residual[0] = T(0.0);

		if ( (angle_ugv1 < bound1_ugv) || (angle_ugv1 > bound2_ugv) ) 
			residual[1] =  wf_ * exp(2.0* value_xy2 );
		else
			residual[1] = T(0.0);

		return true;
	}

	double wf_, ang_;

private:

};


#endif