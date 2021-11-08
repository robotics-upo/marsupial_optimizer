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
  	KinematicsFunctorUGV(double weight_factor, double angle_bound)
	: wf_(weight_factor), ang_(angle_bound) 
	{}

  	template <typename T>
  	bool operator()(const T* const statePos1, const T* const statePos2, const T* const statePos3, T* residual) const 
	{
		// Kinematics for ugv XY Axes
		T vector1[2] = {statePos2[1]-statePos1[1],statePos2[2]-statePos1[2]};
		T vector2[2] = {statePos3[1]-statePos2[1],statePos3[2]-statePos2[2]};
		T dot_product = (vector2[0] * vector1[0]) + (vector2[1] * vector1[1]);
	
		//Compute norm of vectors
		T arg1 = (vector1[0] * vector1[0]) + (vector1[1] * vector1[1]);
		T arg2 = (vector2[0] * vector2[0]) + (vector2[1] * vector2[1]);
		T norm_vector1, norm_vector2, cos_angle;
		
		if (arg1 < 0.0001 && arg1 > -0.0001)
			norm_vector1 = T{0.0};
		else
			norm_vector1 = sqrt(arg1);
		if (arg2 < 0.0001 && arg2 > -0.0001)
			norm_vector2 = T{0.0};
		else
			norm_vector2 = sqrt(arg2);
			
		// Compute cos(angle)	
		if (norm_vector1 < 0.0001 || norm_vector2 < 0.0001)
			cos_angle = T{0.0};
		else
			cos_angle = dot_product/(norm_vector1 * norm_vector2);
		T bound = T{cos(ang_)};
		T b_;
		T max_value_residual = T{100.0};
        T min_value_residual = T{0.0};
        T value_dependent2 = T{-1.0};
        T value_dependent1 = T{bound};
        T m ;
		
        if ( cos_angle > bound){
            m = T{0.0};
			b_ = T{1.0};
		} else{
            m = (max_value_residual- min_value_residual)/(value_dependent2 - value_dependent1);
			b_ = bound;
		}	
		residual[0] =  wf_ * m * (cos_angle - b_);
		
		// std::cout << "KinematicsFunctorUGV residual[0]= "<< residual[0] << " , cos_angle= " << cos_angle << " , b_= " << b_ << std::endl;
		
		return true;
	}

double wf_, ang_;

private:
};

#endif