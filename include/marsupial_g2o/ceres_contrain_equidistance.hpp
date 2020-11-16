#ifndef CERES_CONTRAIN_EQUIDISTANCE_HPP
#define CERES_CONTRAIN_EQUIDISTANCE_HPP


#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;



class EquiDistanceFunctor {

public:
  EquiDistanceFunctor(double weight_factor, double initial_distance): wf_(weight_factor), int_d_(initial_distance) {}

  template <typename T>
  bool operator()(const T* const pose1, const T* const pose2, T* residual) const {
    residual[0] = wf_ * ( (pow(int_d_,2) - pow(pose1[0]-pose2[0],2)) + (pow(int_d_,2) - pow(pose1[1]-pose2[1],2)) + (pow(int_d_,2) - pow(pose1[2]-pose2[2],2)) );
    return true;
  }

 double wf_, int_d_;

 private:
};


#endif