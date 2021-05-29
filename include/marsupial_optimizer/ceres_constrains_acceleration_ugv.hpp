#ifndef CERES_ACCELERATION_VELOCITY_UGV_HPP
#define CERES_ACCELERATION_VELOCITY_UGV_HPP


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
  AccelerationFunctorUGV(double weight_factor, double init_acc_ugv): wf_(weight_factor), ia_ugv_(init_acc_ugv) {}

  template <typename T>
  bool operator()(const T* const statePosUGV1, const T* const statePosUGV2, const T* const statePosUGV3,
				  const T* const stateT1, const T* const stateT2, T* residual) const 
{
	T d1_ugv_ = sqrt( pow(statePosUGV1[1]-statePosUGV2[1],2) + pow(statePosUGV1[2]-statePosUGV2[2],2) + pow(statePosUGV1[2]-statePosUGV2[3],2) );
	T d2_ugv_ = sqrt( pow(statePosUGV2[1]-statePosUGV3[1],2) + pow(statePosUGV2[2]-statePosUGV3[2],2) + pow(statePosUGV2[2]-statePosUGV3[3],2) );
	T v1_ugv_;
	T v2_ugv_;
	T a_ugv_;
	if(stateT1[1] < 0.0001)
		v1_ugv_ = T{0.0};
	else
		v1_ugv_ = d1_ugv_ / (stateT1[1]);
	
	if(stateT2[1] < 0.0001)
		v2_ugv_ = T{0.0};
	else
		v2_ugv_ = d2_ugv_ / (stateT2[1]);
	

	if(stateT1[1] < 0.0001 && stateT2[1]< 0.0001){
  		a_ugv_ = T{0.0};
		residual[0] =  a_ugv_;
	}	
	else{	  
	  	a_ugv_ = (v2_ugv_ - v1_ugv_)/(stateT1[1] + stateT2[1]);
		residual[0] =  wf_ * exp(a_ugv_ - ia_ugv_);
	}

	return true;
}

 double wf_, ia_ugv_, ia_uav_;

 private:
};


#endif