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
  AccelerationFunctor(double weight_factor, double init_acc): wf_(weight_factor), ia_(init_acc) {}

  template <typename T>
  bool operator()(const T* const state1, const T* const state2, const T* const state3, T* residual) const {

	T d1_ = ( pow(state1[0]-state2[0],2) + pow(state1[1]-state2[1],2) + pow(state1[2]-state2[2],2) );
	T d2_ = ( pow(state2[0]-state3[0],2) + pow(state2[1]-state3[1],2) + pow(state2[2]-state3[2],2) );
	T v1_ = d1_ / (state2[3] - state1[3]);
	T v2_ = d2_ / (state3[3] - state1[3]);
  T a_ = (v1_ - v2_)/((state1[3] - state2[3]) + (state2[3] - state3[3]));

	residual[0] =  wf_ * exp( sqrt((a_ - ia_)*(a_ - ia_)));

    return true;
  }

 double wf_, ia_;

 private:
};


#endif