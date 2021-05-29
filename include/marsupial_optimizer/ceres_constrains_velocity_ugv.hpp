#ifndef CERES_CONSTRAINS_VELOCITYUGV_HPP
#define CERES_CONSTRAINS_VELOCITYUGV_HPP


#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;



class VelocityFunctorUGV {

public:
  VelocityFunctorUGV(double weight_factor, double init_vel_): wf_(weight_factor), iv_(init_vel_) {}

  template <typename T>
  bool operator()(const T* const statePos1, const T* const statePos2, const T* const stateT1, T* residual) const 
  {
      T d_ugv_;
      d_ugv_ = sqrt((statePos2[1]-statePos1[1])*(statePos2[1]-statePos1[1]) + 
                    (statePos2[2]-statePos1[2])*(statePos2[2]-statePos1[2]) + 
                    (statePos2[3]-statePos1[3])*(statePos2[3]-statePos1[3]));
      
      if(stateT1[1] < 0.0001)
        residual[0] = T{0.0};
      else
        residual[0] =  wf_ * exp((d_ugv_/stateT1[1]) -iv_);  //To avoid division that make underterminated residual: v*t=d
        // residual[0] =  wf_ * ((stateT1[1])*iv_ - d_ugv_);  //To avoid division that make underterminated residual: v*t=d

      return true;
  }

 double wf_, iv_;

 private:
};


#endif