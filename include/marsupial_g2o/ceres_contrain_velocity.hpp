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
  bool operator()(const T* const state1, const T* const state2, T* residual) const {

	T d_ = sqrt(pow(state2[0]-state1[0],2) + pow(state2[1]-state1[1],2) + pow(state2[2]-state1[2],2)) ;
	T v_ = d_ / (state2[3] - state1[3]);

  std::cout << std::endl << "d_= " << d_ << " , v_ = " << v_ << " , state1=[" << state1[0] << ";" << state1[1] << ";" << state1[2] << ";" << state1[3] << ";" << state1[4] << "] ; , state2=[" 
                                                                 << state2[0] << ";" << state2[1] << ";" << state2[2] << ";" << state2[3] << ";" << state2[4] << std::endl; 



	residual[0] =  wf_ * sqrt((v_ - iv_)*(v_ - iv_));

    return true;
  }

 double wf_, iv_;

 private:
};


#endif