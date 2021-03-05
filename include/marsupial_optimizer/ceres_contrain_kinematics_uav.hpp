#ifndef CERES_CONTRAIN_KINEMATICS_UAV_HPP
#define CERES_CONTRAIN_KINEMATICS_UAV_HPP


#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

class KinematicsFunctorUAV 
{

public:
  KinematicsFunctorUAV(double weight_factor, double angle_bound): wf_(weight_factor), ang_(angle_bound) {}

  template <typename T>
  bool operator()(const T* const statePos1, const T* const statePos2, const T* const statePos3, T* residual) const 
  {
	//Kinematics for uav
	T vector1_uav[3] = {statePos2[4]-statePos1[4],statePos2[5]-statePos1[5],statePos2[6]-statePos1[6]};
	T vector2_uav[3] = {statePos2[4]-statePos3[4],statePos2[5]-statePos3[5],statePos2[6]-statePos3[6]};

	T dot_product_uav = (vector2_uav[0] * vector1_uav[0]) + (vector2_uav[1] * vector1_uav[1]) + (vector2_uav[2] * vector1_uav[2]);
	T norm_vector1_uav = sqrt((vector1_uav[0] * vector1_uav[0]) + (vector1_uav[1] * vector1_uav[1]) + (vector1_uav[2] * vector1_uav[2]));
	T norm_vector2_uav = sqrt((vector2_uav[0] * vector2_uav[0]) + (vector2_uav[1] * vector2_uav[1]) + (vector2_uav[2] * vector2_uav[2]));

	T angle_uav = acos(dot_product_uav / (norm_vector1_uav*norm_vector2_uav));

	T bound1_uav = T(M_PI) - T(ang_);
	T bound2_uav = T(M_PI) + T(ang_);
	T bound_uav = (bound1_uav + bound2_uav)/2.0;	

	if ( (angle_uav < bound1_uav) || (angle_uav > bound2_uav) ) 
		 residual[0] =  wf_ * exp( sqrt((angle_uav - bound_uav)*(angle_uav - bound_uav)));
	else
		 residual[0] = T(0.0);

    return true;
  }

 double wf_, ang_;

 private:
};


#endif