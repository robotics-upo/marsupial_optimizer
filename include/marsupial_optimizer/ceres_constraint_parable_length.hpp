#ifndef CERES_CONSTRAINS_PARABLE_LENGTH_AUTODIFF_HPP
#define CERES_CONSTRAINS_PARABLE_LENGTH_AUTODIFF_HPP

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

class AutodiffParableLengthFunctor {

public:
    AutodiffParableLengthFunctor(){}

	struct ParableLengthFunctor 
	{
	ParableLengthFunctor(double weight_factor, geometry_msgs::Vector3 pos_reel_ugv_, double max_L_, bool write_data, std::string user_name)
					: wf(weight_factor), pos_reel_ugv(pos_reel_ugv_), max_L(max_L_),w_d(write_data), user(user_name)
		{}

		template <typename T>
		bool operator()(const T* const pUGV, const T* const pUAV, const T* const param, T* residual) const 
		{
			T ugv_reel[4] = {pUGV[0], pUGV[1], pUGV[2], pUGV[3] + T{pos_reel_ugv.z}}; // Set first parable point on the reel position
			T dist = T{1.001} * sqrt(pow(pUAV[1]-ugv_reel[1],2)+pow(pUAV[2]-ugv_reel[2],2)+pow(pUAV[3]-ugv_reel[3],2)); 
			T maxL = T{max_L};
		
			// Compute parable L : log(q + ((q + 2*p*x)^2 + 1)^(1/2) + 2*p*x)/(4*p) + ((q + 2*p*x)*((q + 2*p*x)^2 + 1)^(1/2))/(4*p) , x = xA and xB
			T X = T{0.0}; // X is 0.0 because is considered that the parable beginning in the ugv reel
			T val = T{2.0}*param[1]*X+param[2]; // This is a common term for the L equation
			T La = (log( param[2] + sqrt((val*val) + T{1.0}) + T{2.0}*param[1]*X)/(T{4.0}*param[1]) + ((val)*sqrt((val*val) + T{1.0}))/(T{4.0}*param[1]));
			
			X = {sqrt(pow(pUAV[1]-ugv_reel[1],2)+pow(pUAV[2]-ugv_reel[2],2))};
			val = T{2.0}*param[1]*X+param[2];
			T Lb = (log( param[2] + sqrt((val*val) + T{1.0}) + T{2.0}*param[1]*X)/(T{4.0}*param[1]) + ((val)*sqrt((val*val) + T{1.0}))/(T{4.0}*param[1]));
			T L = Lb - La;

			T diff_;
			if (L < dist )
				diff_ = (dist - L);
			else if (L > maxL)
				diff_ = (L - maxL);
			else
				diff_ = T{0.0};

			residual[0] = wf * 100* (exp(diff_)-1.0) ;
					
			return true;
		}
		
		bool w_d;
		double wf, max_L;
		geometry_msgs::Vector3 pos_reel_ugv;
		std::string user;
	};

private:

};

#endif