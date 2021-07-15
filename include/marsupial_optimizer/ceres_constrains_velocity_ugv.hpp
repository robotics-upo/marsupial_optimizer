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
      T d_ugv_, arg_d_ugv_;
      
      arg_d_ugv_ = pow(statePos2[1]-statePos1[1],2) + pow(statePos2[2]-statePos1[2],2) + pow(statePos2[3]-statePos1[3],2);
      
      if (arg_d_ugv_ < 0.001 && arg_d_ugv_ > -0.001)
        d_ugv_ = T{0.0};
      else               
        d_ugv_ = sqrt(arg_d_ugv_);
      
      if(stateT1[1] < 0.001)
        residual[0] = T{0.0};
      else
        residual[0] =  wf_ * ((d_ugv_ / stateT1[1]) - iv_);  //To avoid division that make underterminated residual: v*t=d

      // printf("VelocityFunctorUGV , ");
      // std::cout << "d_ugv_= " << d_ugv_ << " , stateT1[1]= " << stateT1[1] <<std::endl;
      // std::cout << "residual[0]= " << residual[0] << std::endl;

      return true;
  }

 double wf_, iv_;

 private:
};


#endif