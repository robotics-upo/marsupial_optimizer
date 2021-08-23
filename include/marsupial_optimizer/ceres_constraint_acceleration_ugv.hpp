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

class AccelerationFunctorUGV {

public:
  AccelerationFunctorUGV(double weight_factor, double init_acc_ugv, double count_fix_points_ugv): wf_(weight_factor), ia_ugv_(init_acc_ugv), cfp_ugv(count_fix_points_ugv) {}

	template <typename T>
	bool operator()(const T* const statePosUGV1, const T* const statePosUGV2, const T* const statePosUGV3,
					const T* const stateT1, const T* const stateT2, T* residual) const 
	{
		T d1_ugv_, d2_ugv_, arg_d1_ugv_, arg_d2_ugv_;
		T v1_ugv_, v2_ugv_, a_ugv_;

		//Get value distance between pos 1 and pos 2
		arg_d1_ugv_ = pow(statePosUGV1[1]-statePosUGV2[1],2) + pow(statePosUGV1[2]-statePosUGV2[2],2) + pow(statePosUGV1[3]-statePosUGV2[3],2);
		if (arg_d1_ugv_ < 0.001 && arg_d1_ugv_ > -0.001)
			d1_ugv_ = T{0.0};
		else
			d1_ugv_ = sqrt(arg_d1_ugv_);
		
		//Get value distance between pos 2 and pos 3
		arg_d2_ugv_ = pow(statePosUGV2[1]-statePosUGV3[1],2) + pow(statePosUGV2[2]-statePosUGV3[2],2) + pow(statePosUGV2[3]-statePosUGV3[3],2);
		if (arg_d2_ugv_ < 0.001 && arg_d2_ugv_ > -0.001)
			d2_ugv_ = T{0.0};
		else
			d2_ugv_ = sqrt(arg_d2_ugv_);

		if(stateT1[1] < 0.001)
			v1_ugv_ = T{0.0};
		else
			v1_ugv_ = d1_ugv_ / (stateT1[1]);
		
		if(stateT2[1] < 0.001)
			v2_ugv_ = T{0.0};
		else
			v2_ugv_ = d2_ugv_ / (stateT2[1]);
		

		if(stateT1[1] < 0.001 && stateT2[1]< 0.001){
			a_ugv_ = T{0.0};
			residual[0] =  a_ugv_;
		}	
		else{	  
			a_ugv_ = (v2_ugv_ - v1_ugv_)/(stateT1[1] + stateT2[1]);
			residual[0] =  wf_ * (a_ugv_ - ia_ugv_);
			// residual[0] =  wf_ * exp(a_ugv_ - ia_ugv_);
		}
		// std::cout <<"statePosUGV1[0]= " << statePosUGV1[0] <<" , statePosUGV2[0] " << statePosUGV2[0]<<" , statePosUGV3[0] " << statePosUGV3[0]<< std::endl ; 
		// std::cout <<"d1_ugv_= " << d1_ugv_ <<" , d2_ugv_= " << d2_ugv_<< std::endl ; 
		// std::cout <<"stateT1[1]= " << stateT1[1] <<" , stateT2[1]= " << stateT2[1]<< std::endl ; 
		// std::cout <<"v1_ugv_= " << v1_ugv_ <<" , v2_ugv_= " << v2_ugv_<< std::endl ; 
		// std::cout << "a_ugv_= " << a_ugv_ << std::endl; 
		// std::cout <<"residual[0]= " << residual[0] << std::endl;

		return true;
}

 double wf_, ia_ugv_, ia_uav_ , cfp_ugv;

 private:
};

#endif