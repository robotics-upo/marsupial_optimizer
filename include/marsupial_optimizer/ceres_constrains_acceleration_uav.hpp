#ifndef CERES_ACCELERATION_VELOCITY_UAV_HPP
#define CERES_ACCELERATION_VELOCITY_UAV_HPP


#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;



class AccelerationFunctorUAV {

public:
  AccelerationFunctorUAV(double weight_factor, double init_acc_uav): wf_(weight_factor), ia_uav_(init_acc_uav) {}

  template <typename T>
  bool operator()(const T* const statePosUAV1, const T* const statePosUAV2, const T* const statePosUAV3, 
				  const T* const stateT1, const T* const stateT2, T* residual) const 
{
  	T d1_uav_ = sqrt( pow(statePosUAV1[1]-statePosUAV2[1],2) + pow(statePosUAV1[2]-statePosUAV2[2],2) + pow(statePosUAV1[3]-statePosUAV2[3],2) );
	T d2_uav_ = sqrt( pow(statePosUAV2[1]-statePosUAV3[1],2) + pow(statePosUAV2[2]-statePosUAV3[2],2) + pow(statePosUAV2[3]-statePosUAV3[3],2) );
	T v1_uav_ = d1_uav_ / (stateT1[2]);
	T v2_uav_ = d2_uav_ / (stateT2[2]);
  	T a_uav_ = (v2_uav_ - v1_uav_)/(stateT1[2] + stateT2[2]);

	residual[0] =  wf_ * (a_uav_ - ia_uav_);

	return true;
}

 double wf_, ia_uav_;

 private:
};


#endif