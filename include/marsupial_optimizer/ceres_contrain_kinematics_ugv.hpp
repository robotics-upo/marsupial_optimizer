#ifndef CERES_CONTRAIN_KINEMATICS_UGV_HPP
#define CERES_CONTRAIN_KINEMATICS_UGV_HPP


#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"
#include <ros/ros.h>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

class KinematicsFunctorUGV 
{

public:
  	KinematicsFunctorUGV(double weight_factor, double angle_bound): wf_(weight_factor), ang_(angle_bound) {}

  	template <typename T>
  	bool operator()(const T* const statePos1, const T* const statePos2, const T* const statePos3, T* residual) const 
	{
		// Kinematics for ugv
		T vector1_ugv[3] = {statePos2[1]-statePos1[1],statePos2[2]-statePos1[2],statePos2[3]-statePos1[3]};
		T vector2_ugv[3] = {statePos2[1]-statePos3[1],statePos2[2]-statePos3[2],statePos2[3]-statePos3[3]};
		T dot_product_ugv = (vector2_ugv[0] * vector1_ugv[0]) + (vector2_ugv[1] * vector1_ugv[1]) + (vector2_ugv[2] * vector1_ugv[2]);
		T norm_vector1_ugv = sqrt((vector1_ugv[0] * vector1_ugv[0]) + (vector1_ugv[1] * vector1_ugv[1]) + (vector1_ugv[2] * vector1_ugv[2]));
		T norm_vector2_ugv = sqrt((vector2_ugv[0] * vector2_ugv[0]) + (vector2_ugv[1] * vector2_ugv[1]) + (vector2_ugv[2] * vector2_ugv[2]));
		if (norm_vector1_ugv < 0.0000001 || norm_vector2_ugv < 0.0000001 ){
			residual[0] = T(0.0);
			return true;
		}
		else{
			T angle_ugv = acos(dot_product_ugv / (norm_vector1_ugv*norm_vector2_ugv));
			T bound1_ugv = T(M_PI) - T(ang_);
			T bound2_ugv = T(M_PI) + T(ang_);
			T bound_ugv = (bound1_ugv + bound2_ugv)/2.0;
			// std::cout << "angle= "<< angle_ugv <<", dot_product= "<<dot_product_ugv << ", bound1= " << bound1_ugv << ", bound2= " << bound2_ugv << ", bound_ugv = " << bound_ugv <<  std::endl;

			if ( (angle_ugv < bound1_ugv) || (angle_ugv > bound2_ugv) ) 
				residual[0] =  wf_ * exp( sqrt((angle_ugv - bound_ugv)*(angle_ugv - bound_ugv)));
			else
				residual[0] = T(0.0);

			return true;
		}
	}

	double wf_, ang_;

private:

};


#endif