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
	RotationFunctorUGV(double weight_factor): wf_(weight_factor){}

	template <typename T>
	// bool operator()(const T* const statePos1, const T* const statePos2, const T* const stateRot1, T* residual) const 
	bool operator()(const T* const stateRot1, const T* const stateRot2, const T* const stateRot3 , const T* const stateRot4, T* residual) const 
  	{
		// T d_ugv_;
		// d_ugv_ = sqrt((statePos2[1]-statePos1[1])*(statePos2[1]-statePos1[1]) + 
		// 			  (statePos2[2]-statePos1[2])*(statePos2[2]-statePos1[2]) + 
		// 			  (statePos2[3]-statePos1[3])*(statePos2[3]-statePos1[3]));

		double roll1_, pitch1_, yaw1_;
		double roll2_, pitch2_, yaw2_, roll3_, pitch3_, yaw3_, roll4_, pitch4_, yaw4_;
		tf::Quaternion q1_(stateRot1[1],stateRot1[2],stateRot1[3],stateRot1[4]);
		tf::Matrix3x3 M1_(q1_);	
		M1_.getRPY(roll1_, pitch1_, yaw1_);
		tf::Quaternion q2_(stateRot2[1],stateRot2[2],stateRot2[3],stateRot2[4]);
		tf::Matrix3x3 M2_(q2_);	
		M2_.getRPY(roll2_, pitch2_, yaw2_);
		tf::Quaternion q3_(stateRot3[1],stateRot3[2],stateRot3[3],stateRot3[4]);
		tf::Matrix3x3 M3_(q3_);	
		M3_.getRPY(roll3_, pitch3_, yaw3_);
		tf::Quaternion q4_(stateRot4[1],stateRot4[2],stateRot4[3],stateRot4[4]);
		tf::Matrix3x3 M4_(q4_);	
		M4_.getRPY(roll4_, pitch4_, yaw4_);

		// T yaw_;
		// if (d_ugv_ < 0.0001)
		// 	yaw_ = y_;
		// else
		// 	yaw_ = atan2(statePos2[2] - statePos1[2], statePos2[1] - statePos1[1]);

		T diff_angle1_ = yaw1_ - yaw2_;	
		T diff_angle2_ = yaw2_ - yaw3_;	
		T diff_angle3_ = yaw3_ - yaw4_;	
		residual[0] =  wf_ * ( diff_angle1_);
		residual[1] =  wf_ * ( diff_angle2_);
		residual[2] =  wf_ * ( diff_angle3_);

		return true;
	}

	double wf_;

private:

};


#endif