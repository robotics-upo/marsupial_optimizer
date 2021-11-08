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
  AccelerationFunctorUAV(double weight_factor, double init_acc_uav): wf_(weight_factor), ia_(init_acc_uav) {}

  template <typename T>
  bool operator()(const T* const statePos1, const T* const statePos2, const T* const statePos3, 
				  const T* const stateT1, const T* const stateT2, T* residual) const 
{
  	T d1_, d2_, v1_, v2_, a_;

	d1_ = sqrt( pow(statePos1[1]-statePos2[1],2) + pow(statePos1[2]-statePos2[2],2) + pow(statePos1[3]-statePos2[3],2) );
	d2_ = sqrt( pow(statePos2[1]-statePos3[1],2) + pow(statePos2[2]-statePos3[2],2) + pow(statePos2[3]-statePos3[3],2) );

	if(stateT1[1] < 0.0001)
		v1_ = T{0.0};
	else
		v1_ = d1_ / (stateT1[1]);
	
	if(stateT2[1] < 0.0001)
		v2_ = T{0.0};
	else
		v2_ = d2_ / (stateT2[1]);
	

	if(stateT1[1] < 0.0001 && stateT2[1]< 0.0001){
  		a_ = T{0.0};
		residual[0] =  a_;
	}	
	else{	  
	  	a_ = (v2_ - v1_)/(stateT1[1] + stateT2[1]);
		residual[0] =  wf_ * (a_ - ia_);
	}

	// std::cout << "AccelerationFunctorUAV : residual[0]= " << residual[0] << " , a_= " << a_ << " , stateT1[1]= " << stateT1[1]
	// 	 		  << " , stateT2[1]= " << stateT2[1] << " , v1_= " << v1_ <<" , v2_= " << v2_ << std::endl;

	return true;

}

 double wf_, ia_;

 private:
};


#endif