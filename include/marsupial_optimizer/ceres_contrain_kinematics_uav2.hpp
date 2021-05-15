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
	// Kinematics for uav
	T vector1_uav_xy[2] = {statePos2[1]-statePos1[1],statePos2[2]-statePos1[2]};
	T vector2_uav_xy[2] = {statePos2[1]-statePos3[1],statePos2[2]-statePos3[2]};
	T vector1_uav_xz[2] = {statePos2[1]-statePos1[1],statePos2[3]-statePos1[3]};
	T vector2_uav_xz[2] = {statePos2[1]-statePos3[1],statePos2[3]-statePos3[3]};

	T dot_product_uav_xy = (vector2_uav_xy[0] * vector1_uav_xy[0]) + (vector2_uav_xy[1] * vector1_uav_xz[1]) ;
	T norm_vector1_uav_xy = sqrt((vector1_uav_xy[0] * vector1_uav_xy[0]) + (vector1_uav_xy[1] * vector1_uav_xy[1]));
	T norm_vector2_uav_xy = sqrt((vector2_uav_xy[0] * vector2_uav_xy[0]) + (vector2_uav_xy[1] * vector2_uav_xy[1]));
	
	T dot_product_uav_xz = (vector2_uav_xz[0] * vector1_uav_xz[0]) + (vector2_uav_xz[2] * vector1_uav_xz[2]);
	T norm_vector1_uav_xz = sqrt((vector1_uav_xz[0] * vector1_uav_xz[0]) + (vector1_uav_xz[2] * vector1_uav_xz[2]));
	T norm_vector2_uav_xz = sqrt((vector2_uav_xz[0] * vector2_uav_xz[0]) + (vector2_uav_xz[2] * vector2_uav_xz[2]));
	
	T denom_xy = (norm_vector1_uav_xy * norm_vector2_uav_xy);
	T denom_xz = (norm_vector1_uav_xz * norm_vector2_uav_xz);

	T angle_uav_xy;
	T angle_uav_xz;
	
	if(denom_xy < 0.001)
		angle_uav_xy = T(0.0);
	else
		angle_uav_xy = acos(dot_product_uav_xy / (denom_xy));

	if(denom_xz < 0.001)
		angle_uav_xz = T(0.0);
	else
		angle_uav_xz = acos(dot_product_uav_xz / (denom_xz));

	T bound1_uav =  T(-ang_);
	T bound2_uav =  T(ang_);

	if ( (angle_uav_xy < bound1_uav) || (angle_uav_xy > bound2_uav) )
		 residual[0] =  wf_ * (sqrt((angle_uav_xy*angle_uav_xy)));
	else
		 residual[0] = T(0.0);

	if ( (angle_uav_xz < bound1_uav) || (angle_uav_xz > bound2_uav) )
		 residual[1] =  wf_ * (sqrt((angle_uav_xz*angle_uav_xz)));
	else
		 residual[1] = T(0.0);		
		 

    return true;
  }

 double wf_, ang_;

 private:
};


#endif