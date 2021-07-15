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
		T vector1[2] = {statePos2[1]-statePos1[1],statePos2[2]-statePos1[2]};
		T vector2[2] = {statePos3[1]-statePos2[1],statePos3[2]-statePos2[2]};
		T dot_product = (vector2[0] * vector1[0]) + (vector2[1] * vector1[1]);
	
		//Compute norm of vectors
		T arg1 = (vector1[0] * vector1[0]) + (vector1[1] * vector1[1]);
		T arg2 = (vector2[0] * vector2[0]) + (vector2[1] * vector2[1]);
		T norm_vector1;
		T norm_vector2;
		if (arg1 < 0.0001 && arg1 > -0.0001)
			norm_vector1 = T{0.0};
		else
			norm_vector1 = sqrt(arg1);
		if (arg2 < 0.0001 && arg2 > -0.0001)
			norm_vector2 = T{0.0};
		else
			norm_vector2 = sqrt(arg2);
			
		// Compute cos(angle)	
		T cos_angle = dot_product/(norm_vector1 * norm_vector2);
		double bound = cos(ang_);

		//Compute Residual
		if ( cos_angle > bound || norm_vector1 < 0.0001 && norm_vector1 > -0.0001 || norm_vector2 < 0.0001 && norm_vector2 > -0.0001) 
			residual[0] = T(0.0);
		else
			residual[0] =  wf_ * 100.0 * (cos_angle - 1.0);

			return true;
		
	}

	double wf_, ang_;

private:

};


#endif