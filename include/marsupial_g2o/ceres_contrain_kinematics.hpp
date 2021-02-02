#ifndef CERES_CONTRAIN_KINEMATICS_HPP
#define CERES_CONTRAIN_KINEMATICS_HPP


#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;



class KinematicsFunctor {

public:
  KinematicsFunctor(double weight_factor, double angle_bound): wf_(weight_factor), ang_(angle_bound) {}

  template <typename T>
  bool operator()(const T* const state1, const T* const state2, const T* const state3, T* residual) const {

	T vector1[3] = {state2[1]-state1[1],state2[2]-state1[2],state2[3]-state1[3]};
	T vector2[3] = {state2[1]-state3[1],state2[2]-state3[2],state2[3]-state3[3]};

	T dot_product = (vector2[0] * vector1[0]) + (vector2[1] * vector1[1]) + (vector2[2] * vector1[2]);
	T norm_vector1 = sqrt((vector1[0] * vector1[0]) + (vector1[1] * vector1[1]) + (vector1[2] * vector1[2]));
	T norm_vector2 = sqrt((vector2[0] * vector2[0]) + (vector2[1] * vector2[1]) + (vector2[2] * vector2[2]));

	T angle = acos(dot_product / (norm_vector1*norm_vector2));

	T bound1 = T(M_PI) - T(ang_);
	T bound2 = T(M_PI) + T(ang_);
	T bound = (bound1 + bound2)/2.0;

	if ( (angle < bound1) || (angle > bound2) ) 
		 residual[0] =  wf_ * exp( sqrt((angle - bound)*(angle - bound)));
	else
		 residual[0] = T(0.0);

    return true;
  }

 double wf_, ang_;

 private:
};


#endif