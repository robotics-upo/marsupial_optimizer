#ifndef CERES_CONTRAIN_VELOCITY_HPP
#define CERES_CONTRAIN_VELOCITY_HPP


#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;



class VelocityFunctor {

public:
  VelocityFunctor(double weight_factor, double init_vel): wf_(weight_factor), iv_(init_vel) {}

  template <typename T>
  bool operator()(const T* const pose1, const T* const pose2, const T* const time1, T* residual) const {

	T d_ = pow(pose1[0]-pose2[0],2) + pow(pose1[1]-pose2[1],2) + pow(pose1[2]-pose2[2],2) ;
	T v_ = d_ / time1[0];

	residual[0] =  wf_ * exp( sqrt((v_ - iv_)*(v_ - iv_)));

    return true;
  }

 double wf_, iv_;

 private:
};


#endif