#ifndef CERES_CONSTRAINS_PARABOLA_LENGTH_AUTODIFF_HPP
#define CERES_CONSTRAINS_PARABOLA_LENGTH_AUTODIFF_HPP

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
			T den;
			T val, La, Lb, L;
			T Xa = T{0.0}; // X is 0.0 because is considered that the parable beginning in the ugv reel
			T Xb = {sqrt(pow(pUAV[1]-ugv_reel[1],2)+pow(pUAV[2]-ugv_reel[2],2))};
			int flag_ = 0;
			if (param[1] < T{0.001} && param[1] > T{-0.001}){
				T distXY = T{1.001} * sqrt(pow(pUAV[1]-ugv_reel[1],2)+pow(pUAV[2]-ugv_reel[2],2)); 
				L = sqrt( pow(Xa-Xb,2) +  pow(Xa*param[2]+param[3] - (Xb*param[2]+param[3]),2)  );
				flag_ = 1;
			}
			else{
				// Compute parable L : log(q + ((q + 2*p*x)^2 + 1)^(1/2) + 2*p*x)/(4*p) + ((q + 2*p*x)*((q + 2*p*x)^2 + 1)^(1/2))/(4*p) , x = xA and xB
				val = T{2.0}*param[1]*Xa+param[2]; // This is a common term for the L equation
				La = (log( param[2] + sqrt((val*val) + T{1.0}) + T{2.0}*param[1]*Xa)/(T{4.0}*param[1]) + ((val)*sqrt((val*val) + T{1.0}))/(T{4.0}*param[1]));
				
				val = T{2.0}*param[1]*Xb+param[2];
				T Lb = (log( param[2] + sqrt((val*val) + T{1.0}) + T{2.0}*param[1]*Xb)/(T{4.0}*param[1]) + ((val)*sqrt((val*val) + T{1.0}))/(T{4.0}*param[1]));

				L = Lb - La;
				flag_ = 2;
			}
			T K = T{1.0};
			residual[0] = wf * ( exp(K*(dist - L)) + exp(K*(L - maxL)) ) ;
// std::cout << "Len,"<< flag_<<" ["<< param[0] <<"] : R[0]:"  << residual[0] << " param[1]:" << param[1] << std::endl ;
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