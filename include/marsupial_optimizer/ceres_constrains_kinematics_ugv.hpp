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
		// std::cout << "statePos1[0] : " <<  statePos1[0] << " , statePos2[0] : " <<  statePos2[0] << " , statePos3[0] : " <<  statePos3[0] << std::endl;

		// Kinematics for ugv XY Axes
		T vector1_ugv_xy[3] = {statePos2[1]-statePos1[1],statePos2[2]-statePos1[2],statePos2[3]-statePos1[3]};
		T vector2_ugv_xy[3] = {statePos3[1]-statePos2[1],statePos3[2]-statePos2[2],statePos3[3]-statePos2[3]};
		T dot_product_ugv_xy = (vector2_ugv_xy[0] * vector1_ugv_xy[0]) + (vector2_ugv_xy[1] * vector1_ugv_xy[1]) + (vector2_ugv_xy[2] * vector1_ugv_xy[2]);
		T norm_vector1_ugv_xy = sqrt((vector1_ugv_xy[0] * vector1_ugv_xy[0]) + (vector1_ugv_xy[1] * vector1_ugv_xy[1]) + (vector1_ugv_xy[2] * vector1_ugv_xy[2]) );
		T norm_vector2_ugv_xy = sqrt((vector2_ugv_xy[0] * vector2_ugv_xy[0]) + (vector2_ugv_xy[1] * vector2_ugv_xy[1]) + (vector2_ugv_xy[2] * vector2_ugv_xy[2]) );
		T angle_ugv_xy;
		T bound1_ugv_xy = T(-ang_);
		T bound2_ugv_xy = T(ang_);

		T dist_12 = sqrt((statePos1[1]-statePos2[1])*(statePos1[1]-statePos2[1]) + 
						 (statePos1[2]-statePos2[2])*(statePos1[2]-statePos2[2]) + 
						 (statePos1[3]-statePos2[3])*(statePos1[3]-statePos2[3]) );
		
		T dist_13 = sqrt((statePos3[1]-statePos2[1])*(statePos3[1]-statePos2[1]) + 
						 (statePos3[2]-statePos2[2])*(statePos3[2]-statePos2[2]) + 
						 (statePos3[3]-statePos2[3])*(statePos3[3]-statePos2[3]) );

		if (dist_12 < 0.001 || dist_13 < 0.001){
			angle_ugv_xy = T{0.0};
			// printf("angle ugv_xy = 0 : ");
		}
		else{
			angle_ugv_xy = acos(dot_product_ugv_xy / (norm_vector1_ugv_xy*norm_vector2_ugv_xy));
			// printf("angle ugv_xy < 0 : ");
		}
		T value_xy = sqrt(angle_ugv_xy*angle_ugv_xy);
		// std::cout << " dot_product_ugv_xy = " << dot_product_ugv_xy << std::endl;
		// std::cout << "norm_vector1_ugv_xy = " << norm_vector1_ugv_xy << " , norm_vector2_ugv_xy = " << norm_vector2_ugv_xy << std::endl;
		// std::cout << "value_xy = " << value_xy << std::endl;

		// Kinematics for ugv XZ Axes
		// T vector1_ugv_xz[2] = {statePos2[1]-statePos1[1],statePos2[3]-statePos1[3]};
		// T vector2_ugv_xz[2] = {statePos3[1]-statePos2[1],statePos3[3]-statePos2[3]};
		// T dot_product_ugv_xz = (vector2_ugv_xz[0] * vector1_ugv_xz[0]) + (vector2_ugv_xz[1] * vector1_ugv_xz[1]);
		// T norm_vector1_ugv_xz = sqrt((vector1_ugv_xz[0] * vector1_ugv_xz[0]) + (vector1_ugv_xz[1] * vector1_ugv_xz[1]) );
		// T norm_vector2_ugv_xz = sqrt((vector2_ugv_xz[0] * vector2_ugv_xz[0]) + (vector2_ugv_xz[1] * vector2_ugv_xz[1]) );
		// T angle_ugv_xz;
		// T bound1_ugv_xz = T(-ang_);
		// T bound2_ugv_xz = T(ang_);
		// if (norm_vector1_ugv_xz < 0.01 || norm_vector2_ugv_xz < 0.01 ){
		// 	angle_ugv_xz = T{0.0};
		// 	printf("angle ugv_xz = 0 : ");
		// }
		// else{
		// 	angle_ugv_xz = acos(dot_product_ugv_xz / (norm_vector1_ugv_xz*norm_vector2_ugv_xz));
		// 	printf("angle ugv_xz < 0 : ");
		// }
		// T value_xz = sqrt(angle_ugv_xz*angle_ugv_xz);
		// std::cout << " value_xz = " << value_xz << std::endl;


		if ( (angle_ugv_xy < bound1_ugv_xy) || (angle_ugv_xy > bound2_ugv_xy) ) 
			residual[0] =  wf_ * exp(2.0* value_xy );
		else
			residual[0] = T(0.0);

		// if ( (angle_ugv_xz < bound1_ugv_xz) || (angle_ugv_xz > bound2_ugv_xz) ) 
		// 	residual[1] =  wf_ * exp( value_xz );
		// else
		// 	residual[1] = T(0.0);

		// std::cout << " residual[0] = " << residual[0] << std::endl;
		// std::cout << " residual[1] = " << residual[1] << std::endl;


		return true;
	}

	double wf_, ang_;

private:

};


#endif