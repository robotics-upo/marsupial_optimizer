#ifndef CERES_ACCELERATION_VELOCITY_HPP
#define CERES_ACCELERATION_VELOCITY_HPP


#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;



class AccelerationFunctor {

public:
  AccelerationFunctor(double weight_factor, double init_acc_ugv, double init_acc_uav): wf_(weight_factor), ia_ugv_(init_acc_ugv), ia_uav_(init_acc_uav) {}

  template <typename T>
  bool operator()(const T* const statePosUAV1, const T* const statePosUAV2, const T* const statePosUAV3, 
  				  const T* const statePosUGV1, const T* const statePosUGV2, const T* const statePosUGV3,
				  const T* const stateT1, const T* const stateT2, T* residual) const 
{
	T d1_ugv_ = sqrt( pow(statePosUGV1[1]-statePosUGV2[1],2) + pow(statePosUGV1[2]-statePosUGV2[2],2) );
	T d2_ugv_ = sqrt( pow(statePosUGV2[1]-statePosUGV3[1],2) + pow(statePosUGV2[2]-statePosUGV3[2],2) );
	T v1_ugv_;
	if(stateT1[1] < 0.00001)
		v1_ugv_ = 0.0;
	else
		v1_ugv_ = d1_ugv_ / (stateT1[1]);
	
	T v2_ugv_ = d2_ugv_ / (stateT2[1]);
  	T a_ugv_ = (v2_ugv_ - v1_ugv_)/(stateT1[1] + stateT2[1]);

  	T d1_uav_ = sqrt( pow(statePosUAV1[4]-statePosUAV2[4],2) + pow(statePosUAV1[5]-statePosUAV2[5],2) + pow(statePosUAV1[6]-statePosUAV2[6],2) );
	T d2_uav_ = sqrt( pow(statePosUAV2[4]-statePosUAV3[4],2) + pow(statePosUAV2[5]-statePosUAV3[5],2) + pow(statePosUAV2[6]-statePosUAV3[6],2) );
	T v1_uav_ = d1_uav_ / (stateT1[1]);
	T v2_uav_ = d2_uav_ / (stateT2[1]);
  	T a_uav_ = (v2_uav_ - v1_uav_)/(stateT1[1] + stateT2[1]);

	residual[0] =  wf_ * (a_ugv_ - ia_ugv_);
	residual[1] =  wf_ * (a_uav_ - ia_uav_);

	return true;
}

 double wf_, ia_ugv_, ia_uav_;

 private:
};


#endif