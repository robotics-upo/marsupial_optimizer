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
  bool operator()(const T* const pose1, const T* const pose2, const T* const pose3, const T* const time1, const T* const time2,T* residual) const {

	T d1_ = ( pow(pose1[0]-pose2[0],2) + pow(pose1[1]-pose2[1],2) + pow(pose1[2]-pose2[2],2) );
	T d2_ = ( pow(pose2[0]-pose3[0],2) + pow(pose2[1]-pose3[1],2) + pow(pose2[2]-pose3[2],2) );
	T v1_ = d1_ / time1[0];
	T v2_ = d2_ / time2[0];
  T a_ = (v1_ - v2_)/(time1[0] + time2[0]);

	residual[0] =  wf_ * exp( sqrt((a_ - ia_)*(a_ - ia_)));

    return true;
  }

 double wf_, ia_;

 private:
};


#endif