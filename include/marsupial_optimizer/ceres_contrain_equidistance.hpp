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
  EquiDistanceFunctor(double weight_factor, double initial_distance_ugv, double initial_distance_uav): wf_(weight_factor), int_d_ugv(initial_distance_ugv), int_d_uav(initial_distance_uav) {}

  template <typename T>
  bool operator()(const T* const statePos1, const T* const statePos2, T* residual) const {
    residual[0] = wf_ *  1.0/ ( 1.0 + exp( pow(int_d_ugv,2) - ((pow(statePos1[1]-statePos2[1],2)) + pow(statePos1[2]-statePos2[2],2)) ) );
    residual[1] = wf_ *  1.0/ ( 1.0 + exp( pow(int_d_uav,2) - ((pow(statePos1[4]-statePos2[4],2)) + pow(statePos1[5]-statePos2[5],2) + pow(statePos1[6]-statePos2[6],2)) ) );
    return true;
  }

 double wf_, int_d_ugv, int_d_uav;

 private:
};


#endif