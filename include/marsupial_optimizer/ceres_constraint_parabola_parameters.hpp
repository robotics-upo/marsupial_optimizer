#ifndef CERES_CONSTRAINS_PARABOLA_PARAMETERS_AUTODIFF_HPP
#define CERES_CONSTRAINS_PARABOLA_PARAMETERS_AUTODIFF_HPP

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
	ParableParametersFunctor(double weight_factor, geometry_msgs::Point pos_reel_ugv_, bool write_data, std::string user_name)
					: wf(weight_factor), pos_reel_ugv(pos_reel_ugv_), w_d(write_data), user(user_name)
		{}

		template <typename T>
		bool operator()(const T* const pUGV, const T* const pUAV, const T* const param, T* residual) const 
		{
			T ugv_reel[4] = {pUGV[0], pUGV[1], pUGV[2], pUGV[3] + T{pos_reel_ugv.z}}; // Set first parable point on the reel position
			T Xa = T{0.0};
			T Xb = sqrt(pow(pUAV[1]-ugv_reel[1],2)+pow(pUAV[2]-ugv_reel[2],2)); 

			T Pa = (param[1]*Xa*Xa + param[2]*Xa + param[3] - ugv_reel[3]);// residual = p*xa² + q*xa + r - ya
			T Pb = (param[1]*Xb*Xb + param[2]*Xb + param[3] - pUAV[3]    );// residual = p*xb² + q*xb + r - yb

			residual[0] = wf *100.0 *(Pa);
			residual[1] = wf *100.0 *(Pb);
// std::cout << "		Param[1]:"<< param[1]<<" Param[2]:"<< param[2] <<" Param[3]:"<< param[3] <<std::endl ;
// std::cout << "		Pa:"<< Pa<<" Pb:"<< Pb << std::endl ;
// std::cout << "		Param_a:" << param_a << " Param_b:" << param_b << std::endl ;
// std::cout << "Par ["<< param[0]<<"] : R[0]:"  << residual[0] <<" R[1]:"  << residual[1] <<std::endl ;
			return true;
		}
		
		bool w_d;
		double wf;
		geometry_msgs::Point pos_reel_ugv;
		std::string user; 
	};

private:

};

#endif