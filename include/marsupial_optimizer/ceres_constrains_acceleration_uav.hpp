#ifndef CERES_ACCELERATION_VELOCITY_UAV_HPP
#define CERES_ACCELERATION_VELOCITY_UAV_HPP


#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;



class AccelerationFunctorUAV {

public:
  AccelerationFunctorUAV(double weight_factor, double init_acc_uav): wf_(weight_factor), ia_uav_(init_acc_uav) {}

  template <typename T>
  bool operator()(const T* const statePos1, const T* const statePos2, const T* const statePos3, 
				  const T* const stateT1, const T* const stateT2, T* residual) const 
{
  	T d1_uav_ = sqrt( pow(statePos1[1]-statePos2[1],2) + pow(statePos1[2]-statePos2[2],2) + pow(statePos1[3]-statePos2[3],2) );
	T d2_uav_ = sqrt( pow(statePos2[1]-statePos3[1],2) + pow(statePos2[2]-statePos3[2],2) + pow(statePos2[3]-statePos3[3],2) );
	T v1_uav_;
	T v2_uav_;
	T a_uav_;
	if(stateT1[1] < 0.0001)
		v1_uav_ = T{0.0};
	else
		v1_uav_ = d1_uav_ / (stateT1[1]);
	
	if(stateT2[1] < 0.0001)
		v2_uav_ = T{0.0};
	else
		v2_uav_ = d2_uav_ / (stateT2[1]);
	

	if(stateT1[1] < 0.0001 && stateT2[1]< 0.0001){
  		a_uav_ = T{0.0};
		residual[0] =  a_uav_;
	}	
	else{	  
	  	a_uav_ = (v2_uav_ - v1_uav_)/(stateT1[1] + stateT2[1]);
		residual[0] =  wf_ * (a_uav_ - ia_uav_);
	}

	return true;

}

 double wf_, ia_uav_;

 private:
};


#endif