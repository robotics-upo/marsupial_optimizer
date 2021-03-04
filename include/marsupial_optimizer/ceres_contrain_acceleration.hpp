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
  bool operator()(const T* const statePos1, const T* const statePos2, const T* const statePos3, const T* const stateT1, const T* const stateT2, T* residual) const {

	T d1_ugv_ = sqrt( pow(statePos1[1]-statePos2[1],2) + pow(statePos1[2]-statePos2[2],2) );
	T d2_ugv_ = sqrt( pow(statePos2[1]-statePos3[1],2) + pow(statePos2[2]-statePos3[2],2) );
	T v1_ugv_ = d1_ugv_ / (stateT1[1]);
	T v2_ugv_ = d2_ugv_ / (stateT2[1]);
  T a_ugv_ = (v2_ugv_ - v1_ugv_)/(stateT1[1] + stateT2[1]);

  T d1_uav_ = sqrt( pow(statePos1[4]-statePos2[4],2) + pow(statePos1[5]-statePos2[5],2) + pow(statePos1[6]-statePos2[6],2) );
	T d2_uav_ = sqrt( pow(statePos2[4]-statePos3[4],2) + pow(statePos2[5]-statePos3[5],2) + pow(statePos2[6]-statePos3[6],2) );
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