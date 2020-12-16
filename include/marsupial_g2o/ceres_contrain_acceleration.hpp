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
  bool operator()(const T* const state1, const T* const state2, const T* const state3, const T* const state4, const T* const state5, T* residual) const {

	T d1_ = sqrt( pow(state1[1]-state2[1],2) + pow(state1[2]-state2[2],2) + pow(state1[3]-state2[3],2) );
	T d2_ = sqrt( pow(state2[1]-state3[1],2) + pow(state2[2]-state3[2],2) + pow(state2[3]-state3[3],2) );
	T v1_ = d1_ / (state4[1]);
	T v2_ = d2_ / (state5[1]);
  T a_ = (v2_ - v1_)/(state4[1] + state5[1]);

	residual[0] =  wf_ * (a_ - ia_);

    return true;
  }

 double wf_, ia_;

 private:
};


#endif