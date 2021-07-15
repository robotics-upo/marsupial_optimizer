#ifndef CERES_CONSTRAINS_KINEMATICS_UAV_HPP
#define CERES_CONSTRAINS_KINEMATICS_UAV_HPP


#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

class KinematicsFunctorUAV 
{

public:
  KinematicsFunctorUAV(double weight_factor, double angle_bound): wf_(weight_factor), ang_(angle_bound) {}

  	template <typename T>
 	bool operator()(const T* const statePos1, const T* const statePos2, const T* const statePos3, T* residual) const 
  	{
		// Kinematics for ugv XY Axes
		T vector1[3] = {statePos2[1]-statePos1[1],statePos2[2]-statePos1[2],statePos2[3]-statePos1[3]};
		T vector2[3] = {statePos3[1]-statePos2[1],statePos3[2]-statePos2[2],statePos3[3]-statePos2[3]};
		T dot_product = (vector2[0] * vector1[0]) + (vector2[1] * vector1[1]) + (vector2[2] * vector1[2]);
	
		//Compute norm of vectors
		T arg1 = (vector1[0] * vector1[0]) + (vector1[1] * vector1[1]) + (vector1[2] * vector1[2]);
		T arg2 = (vector2[0] * vector2[0]) + (vector2[1] * vector2[1]) + (vector2[2] * vector2[2]);
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
		if ( cos_angle > bound) 
			residual[0] = T(0.0);
		else
			residual[0] =  wf_ * 100.0 * (cos_angle - 1.0);
		
		// std::cout<< "KinematicsFunctorUAV" << std::endl;
		// std::cout<< "cos_angle= " << cos_angle <<" , bound= " << bound << std::endl;
		// std::cout<< "residual[0]= " << residual[0] << std::endl;

		return true;
  	}

 double wf_, ang_;

 private:
};


#endif