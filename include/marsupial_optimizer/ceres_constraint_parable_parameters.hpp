#ifndef CERES_CONSTRAINS_PARABLE_PARAMETERS_AUTODIFF_HPP
#define CERES_CONSTRAINS_PARABLE_PARAMETERS_AUTODIFF_HPP

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

#include <iostream>
#include <fstream>
#include <string>

using ceres::AutoDiffCostFunction;
using ceres::NumericDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

class AutodiffParableParametersFunctor {

public:
    AutodiffParableParametersFunctor(){}

	struct ParableParametersFunctor 
	{
	ParableParametersFunctor(double weight_factor, geometry_msgs::Vector3 pos_reel_ugv_, bool write_data, std::string user_name)
					: wf(weight_factor), pos_reel_ugv(pos_reel_ugv_), w_d(write_data), user(user_name)
		{}

		template <typename T>
		bool operator()(const T* const pUGV, const T* const pUAV, const T* const param, T* residual) const 
		{
			T ugv_reel[4] = {pUGV[0], pUGV[1], pUGV[2], pUGV[3] + T{pos_reel_ugv.z}}; // Set first parable point on the reel position
		
			T Xa = T{0.0};
			T Xb = sqrt(pow(pUAV[1]-ugv_reel[1],2)+pow(pUAV[2]-ugv_reel[2],2)); 

			// For Residual, is consider that reel_pos is the origin in the parable plane
			// residual[0] = (wf * 100.0 )* (param[1] * Xa*Xa + param[2] * Xa+ param[3] - ugv_reel[3]);  
			// residual[1] = (wf * 100.0 )* (param[1] * Xb*Xb + param[2] * Xb+ param[3] - pUAV[3]    );  	 
			T pow_a = sqrt((param[1]*Xa*Xa + param[2]*Xa + param[3] - ugv_reel[3])*(param[1]*Xa*Xa + param[2]*Xa+ param[3] - ugv_reel[3]));// residual = p*xa² + q*xa + r - ya
			T pow_b = sqrt((param[1]*Xb*Xb + param[2]*Xb + param[3] - pUAV[3]    )*(param[1]*Xb*Xb + param[2]*Xb+ param[3] - pUAV[3]    ));// residual = p*xb² + q*xb + r - yb
			residual[0] = (wf * 10.0 )* (exp(pow_a) - 1.0);
			residual[1] = (wf * 10.0 )* (exp(pow_b) - 1.0);

			return true;
		}
		
		bool w_d;
		double wf;
		geometry_msgs::Vector3 pos_reel_ugv;
		std::string user;
	};

private:

};

#endif