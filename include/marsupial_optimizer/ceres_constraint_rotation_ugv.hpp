#ifndef CERES_CONSTRAINS_ROTATION_UGV_HPP
#define CERES_CONSTRAINS_ROTATION_UGV_HPP


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
	RotationFunctorUGV(double weight_factor , double yaw): wf_(weight_factor), y_(yaw){}

	template <typename T>
	bool operator()(const T* const statePos1, const T* const statePos2, const T* const stateRot1, T* residual) const 
  	{
		T arg_d_ugv_, d_ugv_;
		arg_d_ugv_ = (statePos2[1]-statePos1[1])*(statePos2[1]-statePos1[1]) + 
					 (statePos2[2]-statePos1[2])*(statePos2[2]-statePos1[2]) + 
					 (statePos2[3]-statePos1[3])*(statePos2[3]-statePos1[3]);
		if (arg_d_ugv_ < 0.0001 && arg_d_ugv_ > -0.0001)
			d_ugv_ = T{0.0};
		else
			d_ugv_ = sqrt(arg_d_ugv_);

		double roll1_, pitch1_, yaw1_;
		tf::Quaternion q1_(stateRot1[1],stateRot1[2],stateRot1[3],stateRot1[4]);
		tf::Matrix3x3 M1_(q1_);	
		M1_.getRPY(roll1_, pitch1_, yaw1_);

		T yaw_;
		if (d_ugv_ < 0.001)
			yaw_ = y_;
		else
			yaw_ = atan2(statePos2[2] - statePos1[2], statePos2[1] - statePos1[1]);
		
		T arg_diff_angle_ = (yaw1_ - yaw_)*(yaw1_ - yaw_);	
	

		residual[0] =  wf_ * exp( arg_diff_angle_);

		return true;
	}

	double wf_ , y_;

private:

};


#endif