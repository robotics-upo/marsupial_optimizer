#ifndef CERES_CONSTRAINS_TETHER_PARAMETERS_AUTODIFF_HPP
#define CERES_CONSTRAINS_TETHER_PARAMETERS_AUTODIFF_HPP

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

#include <iostream>
#include <fstream>
#include <string>
#include <geometry_msgs/Vector3.h>

using ceres::AutoDiffCostFunction;
using ceres::NumericDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

class AutodiffTetherParametersFunctor {

public:
    AutodiffTetherParametersFunctor(){}

	struct TetherParametersFunctor 
	{
	TetherParametersFunctor(double weight_factor, geometry_msgs::Vector3 pos_reel_ugv_, bool write_data, std::string user_name)
					: wf(weight_factor), pos_reel_ugv(pos_reel_ugv_), w_d(write_data), user(user_name)
		{}

		template <typename T>
		bool operator()(const T* const pUGV, const T* const pUAV, const T* const params, T* residual) const 
		{
			T ugv_reel[4] = {pUGV[0], pUGV[1], pUGV[2], pUGV[3] + T{pos_reel_ugv.z}}; // Set first parable point on the reel position
		
			T Xa = T{0.0};
			T Ya = ugv_reel[3];
			T Xb = sqrt(pow(pUAV[1]-ugv_reel[1],2)+pow(pUAV[2]-ugv_reel[2],2)); 
			T Yb = pUAV[3];
			T r1, r2, r3;
			
			// T length = params[3] * sinh(params[1]/params[3]) + params[3] * sinh((Xb - params[1])/params[3]);

			/* Map:
			params[1] = Xo 
			params[2] = Yo 
			params[3] = a 
			params[4] = length (Not used length because it is not optimized)
			*/
			r1 = params[3]*cosh((Xa - params[1])/params[3]) + (params[2]-params[3]) - Ya;//Eq. point A(UGV): residual[0] = a cosh(-Xo/a) + Yo
			r2 = params[3]*cosh((Xb - params[1])/params[3]) + (params[2]-params[3]) - Yb;//Eq. point B(UAV): residual[1] = a cosh(Xb-Xo/a) + Yo - Yb
			// r3 = params[3]*sinh((Xa - params[1])/params[3]) + params[3]*sinh((Xb - params[1])/params[3]) - params[4]; //Eq. length catenary: residual[2] = a cosh(Xb-Xo/a) + a cosh(Xb-Xo/a)

			residual[0] = wf* ((r1));
			residual[1] = wf* ((r2));
			// residual[2] = (wf * 100.0 )* ((r3));

// std::cout << "		["<< params[0]<<"] AutodiffTetherParametersFunctor: residual[0]= " << residual[0] << " , residual[1]= " << residual[1]  << std::endl;

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