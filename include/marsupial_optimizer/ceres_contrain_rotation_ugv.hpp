#ifndef CERES_CONTRAIN_ROTATION_UGV_HPP
#define CERES_CONTRAIN_ROTATION_UGV_HPP


#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"
#include <ros/ros.h>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

class RotationFunctorUGV 
{

public:
	RotationFunctorUGV(double weight_factor, double angle_bound): wf_(weight_factor), ang_(angle_bound) {}

	template <typename T>
	bool operator()(const T* const stateRot1, const T* const stateRot2, T* residual) const 
  	{
		double roll1_, pitch1_, yaw1_, roll2_, pitch2_, yaw2_;
		T sRot1[4] = {stateRot1[1],stateRot1[2],stateRot1[3],stateRot1[4]};
		tf::Quaternion q1_(sRot1[0],sRot1[1],sRot1[2],sRot1[3]);
		tf::Matrix3x3 M1_(q1_);	
		M1_.getRPY(roll1_, pitch1_, yaw1_);

		T sRot2[4] = {stateRot2[1],stateRot2[2],stateRot2[3],stateRot2[4]};
		tf::Quaternion q2_(sRot2[0],sRot2[1],sRot2[2],sRot2[3]);
		tf::Matrix3x3 M2_(q2_);	
		M1_.getRPY(roll2_, pitch2_, yaw2_);

		T diff_angle_ = T(yaw1_) - T(yaw2_);	

		if ( (ang_ < diff_angle_) || (diff_angle_ < -1.0 * ang_) ) 
			 residual[0] =  wf_ * exp( sqrt(diff_angle_*diff_angle_));
		else
			 residual[0] = T(0.0);

		return true;
	}

	double wf_, ang_;

private:

};


#endif