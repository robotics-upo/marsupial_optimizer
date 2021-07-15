#ifndef CERES_CONSTRAINS_VELOCITY_UAV_HPP
#define CERES_CONSTRAINS_VELOCITY_UAV_HPP


#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;



class VelocityFunctorUAV {

public:
  VelocityFunctorUAV(double weight_factor, double init_vel_): wf_(weight_factor), iv_(init_vel_) {}

  template <typename T>
  bool operator()(const T* const statePos1, const T* const statePos2, const T* const stateT, T* residual) const 
  {
      T d_uav_, arg_d_uav_;

      arg_d_uav_ = pow(statePos2[1]-statePos1[1],2) + pow(statePos2[2]-statePos1[2],2) + pow(statePos2[3]-statePos1[3],2) ;

      if(arg_d_uav_ < 0.0001 && arg_d_uav_ > -0.0001)
        d_uav_ = T{0.0};
      else  
        d_uav_ = sqrt(arg_d_uav_);
      
      if(stateT[1] < 0.001)
        residual[0] = T{0.0};
      else
        residual[0] =  wf_ * ((d_uav_ / stateT[1]) - iv_);
      
      return true;
  }

 double wf_, iv_;

 private:
};


#endif