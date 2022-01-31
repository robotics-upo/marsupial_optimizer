#ifndef CERES_CONSTRAINS_SMOOTHNESS_UAV_HPP
#define CERES_CONSTRAINS_SMOOTHNESS_UAV_HPP


#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

#include <iostream>
#include <fstream>
#include <string>

class SmoothnessFunctorUAV 
{

public:
  SmoothnessFunctorUAV(double weight_factor, double angle_bound, bool write_data)
  : wf_(weight_factor), ang_(angle_bound), w_d_(write_data) 
  {}

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
		T max_value_residual = T{20.0};
        T min_value_residual = T{0.0};
        T value_dependent2 = T{-1.0};
        T value_dependent1 = T{1.0};
        T m ;
		
    
		m = (max_value_residual- min_value_residual)/(value_dependent2 - value_dependent1);
		
		residual[0] =  wf_ *( m * (cos_angle - value_dependent1) + min_value_residual);

		// std::cout << "SmoothnessFunctorUAV: residual[0]= "<< residual[0] << " , cos_angle= " << cos_angle << " , b_= " << b_ << std::endl;
		
		if(w_d_){
			std::ofstream ofs;
			std::string name_output_file = "/home/simon/residuals_optimization_data/smoothness_uav.txt";
			ofs.open(name_output_file.c_str(), std::ofstream::app);
			if (ofs.is_open()) 
				ofs << residual[0] << "/" <<std::endl;
			ofs.close();
		}

		return true;
  	}

 bool w_d_;
 double wf_, ang_;

 private:
};


#endif