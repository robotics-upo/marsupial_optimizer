#ifndef CERES_CONSTRAINS_KINEMATICS_UGV_HPP
#define CERES_CONSTRAINS_KINEMATICS_UGV_HPP


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
		T dot_product_ugv, arg_norm_vector1_ugv, arg_norm_vector2_ugv, norm_vector1_ugv, norm_vector2_ugv; 
		T angle_ugv, bound1_ugv, bound2_ugv;
		// Kinematics for ugv XY Axes
		T vector1_ugv[2] = {statePos2[1]-statePos1[1],statePos2[2]-statePos1[2]};
		T vector2_ugv[2] = {statePos3[1]-statePos2[1],statePos3[2]-statePos2[2]};
		dot_product_ugv = (vector2_ugv[0] * vector1_ugv[0]) + (vector2_ugv[1] * vector1_ugv[1]);
		
		arg_norm_vector1_ugv = (vector1_ugv[0] * vector1_ugv[0]) + (vector1_ugv[1] * vector1_ugv[1]);
		if(arg_norm_vector1_ugv < 0.0001 && arg_norm_vector1_ugv > -0.0001)
			norm_vector1_ugv = T{0.0};
		else
			norm_vector1_ugv = sqrt(arg_norm_vector1_ugv);
		
		arg_norm_vector2_ugv = (vector2_ugv[0] * vector2_ugv[0]) + (vector2_ugv[1] * vector2_ugv[1]);
		if(arg_norm_vector2_ugv < 0.0001 && arg_norm_vector2_ugv > -0.0001)
			norm_vector2_ugv = T{0.0};
		else
			norm_vector2_ugv = sqrt(arg_norm_vector2_ugv);

		bound1_ugv = T(-ang_);
		bound2_ugv = T(ang_);

		T dist_12 , dist_13;
		T arg_dist_12, arg_dist_13;
		T value;

		arg_dist_12 = (statePos1[1]-statePos2[1])*(statePos1[1]-statePos2[1]) + 
			 		  (statePos1[2]-statePos2[2])*(statePos1[2]-statePos2[2]) + 
					  (statePos1[3]-statePos2[3])*(statePos1[3]-statePos2[3]) ;
		if(arg_dist_12 < 0.0001 && arg_dist_12 > -0.0001)
			dist_12 = T{0.0};
		else
		dist_12 = sqrt(arg_dist_12);
		
		
		arg_dist_13 = (statePos3[1]-statePos2[1])*(statePos3[1]-statePos2[1]) + 
					  (statePos3[2]-statePos2[2])*(statePos3[2]-statePos2[2]) + 
					  (statePos3[3]-statePos2[3])*(statePos3[3]-statePos2[3]) ;
		if(arg_dist_13 < 0.0001 && arg_dist_13 > -0.0001)
			dist_13 = T{0.0};
		else
			dist_13 = sqrt(dist_13);


		T arg_acos = dot_product_ugv / (norm_vector1_ugv*norm_vector2_ugv);
		if (dist_12 < 0.001 || dist_13 < 0.001 || arg_acos > 1.000 || arg_acos < -1.000)
			angle_ugv = T{0.0};
		else
			angle_ugv = acos(arg_acos);

		if (angle_ugv < 0.0001 && angle_ugv > -0.0001)
			value = T{0.0};
		else
			value = sqrt(angle_ugv*angle_ugv);

			

		if ( (angle_ugv < bound1_ugv) || (angle_ugv > bound2_ugv) ) 
			residual[0] =  wf_ * exp(2.0* value );
		else
			residual[0] = T(0.0);

		

		return true;
	}

	double wf_, ang_;

private:

};


#endif