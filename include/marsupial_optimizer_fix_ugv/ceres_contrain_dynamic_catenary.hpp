#ifndef CERES_CONTRAIN_DYNAMIC_CATENARY_FIX_UGV_HPP
#define CERES_CONTRAIN_DYNAMIC_CATENARY_FIX_UGV_HPP


#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;



class DynamicCatenaryFunctor {

public:
  DynamicCatenaryFunctor(double weight_factor, double dynamic_catenary): wf_(weight_factor), dc_(dynamic_catenary) {}

  template <typename T>
  bool operator()(const T* const state1, const T* const state2, T* residual) const {
    T sl_ = state1[1] * 0.2;
    residual[0] = wf_ *  1.0 / exp (-1.0* pow(state2[1] - state1[1],2)/(2.0 *sl_ * sl_));
    return true;
  }

 double wf_, init_l_, dc_;

 private:
};


#endif