#ifndef CERES_CONTRAIN_TIME_HPP
#define CERES_CONTRAIN_TIME_HPP


#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;



class TimeFunctor {

public:
  TimeFunctor(double weight_factor, double init_time_ugv, double init_time_uav): wf_(weight_factor), it_ugv_(init_time_ugv), it_uav_(init_time_uav) {}

  template <typename T>
  bool operator()(const T* const stateT, T* residual) const 
  {
    residual[0] =  wf_ * (stateT[1] - it_ugv_);
    residual[1] =  wf_ * (stateT[2] - it_uav_);

    return true;
  }

 double wf_, it_ugv_, it_uav_;

 private:
};


#endif