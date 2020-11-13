#pragma once

#ifndef CERES_CONSTRAIN_EQUIDISTANCE_HPP
#define CERES_CONSTRAIN_EQUIDISTANCE_HPP

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

double factor;

struct EquiDistanceFunctor {
  template <typename T>
  bool operator()(const T* const pos1, const T* const pos2, const T* const pos3, const T* const pos4, T* residual) const {
    residual[0] = factor * ( (10.0 - (pos1[0]-pos2[0]).norm() ) + (10.0 - (pos2[0]-pos3[0]).norm()) + (10.0 - (pos3[0]-pos4[0]).norm()) );
    return true;
  }
};

#endif