#ifndef CERES_CONSTRAINT_ACCELERATION_UGV_HPP
#define CERES_CONSTRAINT_ACCELERATION_UGV_HPP


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

class AccelerationFunctorUGV {

public:
  AccelerationFunctorUGV(double weight_factor, double init_acc_ugv, double count_fix_points_ugv): wf_(weight_factor), ia_ugv_(init_acc_ugv), cfp_ugv(count_fix_points_ugv) {}

	template <typename T>
	bool operator()(const T* const statePosUGV1, const T* const statePosUGV2, const T* const statePosUGV3,
					const T* const stateT1, const T* const stateT2, T* residual) const 
	{
		T d1_, d2_, arg_d1_, arg_d2_;
		T v1_, v2_, a_;

		//Get value distance between pos 1 and pos 2
		arg_d1_ = pow(statePosUGV1[1]-statePosUGV2[1],2) + pow(statePosUGV1[2]-statePosUGV2[2],2) + pow(statePosUGV1[3]-statePosUGV2[3],2);
		if (arg_d1_ < 0.001 && arg_d1_ > -0.001)
			d1_ = T{0.0};
		else
			d1_ = sqrt(arg_d1_);
		
		//Get value distance between pos 2 and pos 3
		arg_d2_ = pow(statePosUGV2[1]-statePosUGV3[1],2) + pow(statePosUGV2[2]-statePosUGV3[2],2) + pow(statePosUGV2[3]-statePosUGV3[3],2);
		if (arg_d2_ < 0.001 && arg_d2_ > -0.001)
			d2_ = T{0.0};
		else
			d2_ = sqrt(arg_d2_);

		if(stateT1[1] < 0.001)
			v1_ = T{0.0};
		else
			v1_ = d1_ / (stateT1[1]);
		
		if(stateT2[1] < 0.001)
			v2_ = T{0.0};
		else
			v2_ = d2_ / (stateT2[1]);
		

		if(stateT1[1] < 0.001 && stateT2[1]< 0.001){
			a_ = T{0.0};
			residual[0] =  a_;
		}	
		else{	  
			a_ = (v2_ - v1_)/(stateT1[1] + stateT2[1]);
			residual[0] =  wf_ * (a_ - ia_ugv_);
			// residual[0] =  wf_ * exp(a_ - ia_ugv_);
		}
		// std::cout << "AccelerationFunctorUGV : residual[0]= " << residual[0] << " , a_= " << a_ << " , stateT1[1]= " << stateT1[1]
		//  		  << " , stateT2[1]= " << stateT2[1] << " , v1_= " << v1_ <<" , v2_=" << v2_ << std::endl;

		std::ofstream ofs;
		std::string name_output_file = "/home/simon/residuals_optimization_data/acceleration_ugv.txt";
		ofs.open(name_output_file.c_str(), std::ofstream::app);
		if (ofs.is_open()) 
			ofs << residual[0] << ";" <<std::endl;
		ofs.close();

		return true;
}

 double wf_, ia_ugv_, ia_uav_ , cfp_ugv;

 private:
};

#endif