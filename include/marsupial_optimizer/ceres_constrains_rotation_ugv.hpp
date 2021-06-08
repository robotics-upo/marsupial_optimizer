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
		T d_ugv_;
		d_ugv_ = sqrt((statePos2[1]-statePos1[1])*(statePos2[1]-statePos1[1]) + 
					  (statePos2[2]-statePos1[2])*(statePos2[2]-statePos1[2]) + 
					  (statePos2[3]-statePos1[3])*(statePos2[3]-statePos1[3]));

		double roll1_, pitch1_, yaw1_;
		tf::Quaternion q1_(stateRot1[1],stateRot1[2],stateRot1[3],stateRot1[4]);
		tf::Matrix3x3 M1_(q1_);	
		M1_.getRPY(roll1_, pitch1_, yaw1_);

		// std::cout << "stateRot1[0] = " << stateRot1[0] << std::endl;	
		T yaw_;
		if (d_ugv_ < 0.0001){
			yaw_ = y_;
			// std::cout << "yaw (d_ugv_ < 0.0001)= " << yaw_ << std::endl;
		}
		else{
			yaw_ = atan2(statePos2[2] - statePos1[2], statePos2[1] - statePos1[1]);
			// std::cout << "yaw (atan2)= " << yaw_ << std::endl;
		}
		T diff_angle1_ = sqrt( (yaw1_ - yaw_)*(yaw1_ - yaw_) );	
		// std::cout << "yaw1_= "  << yaw1_ << " , diff_angle1_= " << diff_angle1_ << std::endl;
	

		residual[0] =  wf_ * exp( diff_angle1_);

		return true;
	}

	double wf_ , y_;

private:

};


#endif