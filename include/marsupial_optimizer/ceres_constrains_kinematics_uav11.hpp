#ifndef CERES_CONSTRAINS_KINEMATICS_UAV_HPP
#define CERES_CONSTRAINS_KINEMATICS_UAV_HPP


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
		// Kinematics for ugv XY Axes
		T vector1_uav_xy[2] = {statePos2[1]-statePos1[1],statePos2[2]-statePos1[2]};
		T vector2_uav_xy[2] = {statePos3[1]-statePos2[1],statePos3[2]-statePos2[2]};
		T vector1_uav_xz[2] = {statePos2[1]-statePos1[1],statePos2[3]-statePos1[3]};
		T vector2_uav_xz[2] = {statePos3[1]-statePos2[1],statePos3[3]-statePos2[3]};
		// std::cout << "\nvector1_xy[0]= " << vector1_uav_xy[0] << " , vector1_xy[1]= " << vector1_uav_xy[1] << " , vector2_xy[0]="<< vector2_uav_xy[0] << ", vector2_xy[1]="<< vector2_uav_xy[1] << std::endl;
		// std::cout << "vector1_xz[0]= " << vector1_uav_xz[0] << " , vector1_xz[1]= " << vector1_uav_xz[1] << " , vector2_xz[0]="<< vector2_uav_xz[0] << ", vector2_xz[1]="<< vector2_uav_xz[1] << std::endl;
		T dot_product_uav_xy = (vector2_uav_xy[0] * vector1_uav_xy[0]) + (vector2_uav_xy[1] * vector1_uav_xy[1]);
		T dot_product_uav_xz = (vector2_uav_xy[0] * vector1_uav_xz[0]) + (vector2_uav_xy[2] * vector1_uav_xz[2]);


		if (dot_product_uav_xy < 0.0001 || dot_product_uav_xz < 0.0001)	{
			residual[0] = T(0.0);
			residual[1] = T(0.0);
			return true;
		}
		else{
			T arg1_uav_xy = (vector1_uav_xy[0] * vector1_uav_xy[0]) + (vector1_uav_xy[1] * vector1_uav_xy[1]);
			T arg2_uav_xy = (vector2_uav_xy[0] * vector2_uav_xy[0]) + (vector2_uav_xy[1] * vector2_uav_xy[1]);
			T arg1_uav_xz = (vector1_uav_xz[0] * vector1_uav_xz[0]) + (vector1_uav_xz[1] * vector1_uav_xz[1]);
			T arg2_uav_xz = (vector2_uav_xz[0] * vector2_uav_xz[0]) + (vector2_uav_xz[1] * vector2_uav_xz[1]);
			T norm_vector1_uav_xy;
			T norm_vector2_uav_xy;
			T norm_vector1_uav_xz;
			T norm_vector2_uav_xz;
			if (arg1_uav_xy < 0.0001 && arg1_uav_xy > -0.0001)
				norm_vector1_uav_xy = T{0.0};
			else
				norm_vector1_uav_xy = sqrt(arg1_uav_xy);
			
			if (arg2_uav_xy < 0.0001 && arg2_uav_xy > -0.0001)
				norm_vector1_uav_xy = T{0.0};
			else
				norm_vector2_uav_xy = sqrt(arg2_uav_xy);
			
			if (arg1_uav_xz < 0.0001 && arg1_uav_xz > -0.0001)
				norm_vector1_uav_xz = T{0.0};
			else
				norm_vector1_uav_xz = sqrt(arg1_uav_xz);
			
			if (arg2_uav_xz < 0.0001 && arg2_uav_xz > -0.0001)
				norm_vector2_uav_xz = T{0.0};
			else
				norm_vector2_uav_xz = sqrt(arg2_uav_xz);

			T angle_uav_xy;
			T angle_uav_xz;
			T bound1_uav = T(-ang_);
			T bound2_uav = T(ang_);

			if (norm_vector1_uav_xy < 0.0001 || norm_vector2_uav_xy < 0.0001 || (dot_product_uav_xy / (norm_vector1_uav_xy*norm_vector2_uav_xy)) >= 1.0000 || (dot_product_uav_xy / (norm_vector1_uav_xy*norm_vector2_uav_xy)) < -1.0000)
				angle_uav_xy = T{0.0};
			else
				angle_uav_xy = acos(dot_product_uav_xy / (norm_vector1_uav_xy*norm_vector2_uav_xy));
			
			if (norm_vector1_uav_xz < 0.0001 || norm_vector2_uav_xz < 0.0001 || (dot_product_uav_xz / (norm_vector1_uav_xz*norm_vector2_uav_xz)) >= 1.0000 || (dot_product_uav_xz / (norm_vector1_uav_xz*norm_vector2_uav_xz)) < -1.0000)
				angle_uav_xz = T{0.0};
			else
				angle_uav_xz = acos(dot_product_uav_xz / (norm_vector1_uav_xz*norm_vector2_uav_xz));

			T value_xy;
			T value_xz;
			if(angle_uav_xy < 0.0001 && angle_uav_xy > -0.0001)
				value_xy = T{0.0};
			else
				value_xy = sqrt(angle_uav_xy*angle_uav_xy);
			
			if(angle_uav_xz < 0.0001 && angle_uav_xz > -0.0001)
				value_xz = T{0.0};
			else
				value_xz = sqrt(angle_uav_xz*angle_uav_xz);

			if ( (angle_uav_xy < bound1_uav) || (angle_uav_xy > bound2_uav) ) 
				residual[0] =  wf_ * exp(2.0*(value_xy-bound2_uav));
			else
				residual[0] = T(0.0);
			
			if ( (angle_uav_xz < bound1_uav) || (angle_uav_xz > bound2_uav) ) 
				residual[1] =  wf_ * exp(2.0*(value_xz-bound2_uav));
			else
				residual[1] = T(0.0);

			// std::cout << "vector1_uav_xy[0] = " << vector1_uav_xy[0] << " , vector1_uav_xy[1]= " << vector1_uav_xy[1] << " , vector2_uav_xy[0]= " << vector2_uav_xy[0] << " , vector2_uav_xy[1]= " << vector2_uav_xy[1] << std::endl;
			// std::cout << "vector1_uav_xz[0] = " << vector1_uav_xz[0] << " , vector1_uav_xz[1]= " << vector1_uav_xz[1] << " , vector2_uav_xz[0]= " << vector2_uav_xz[0] << " , vector2_uav_xz[1]= " << vector2_uav_xz[1] << std::endl;
			// std::cout << "arg1_uav_xy= " <<arg1_uav_xy << " , arg2_uav_xy= " << arg2_uav_xy << std::endl;
			// std::cout << "arg1_uav_xz= " <<arg1_uav_xz << " , arg2_uav_xz= " << arg2_uav_xz << std::endl;
			// std::cout << "\nangle_uav_xy = " << angle_uav_xy << " , angle_uav_xz = " << angle_uav_xz << std::endl; 
			// std::cout << "bound1_uav= " << bound1_uav << " , bound2_uav= " << bound2_uav << std::endl;

			// std::cout << "\nKinematic UAV" << std::endl;
			// std::cout << "dot_product_uav_xy = " << dot_product_uav_xy <<  std::endl;
			// std::cout << "norm_vector1_uav_xy= " <<norm_vector1_uav_xy << " , norm_vector2_uav_xy= " << norm_vector2_uav_xy << std::endl;
			// std::cout << "dot_product_uav_xz = " << dot_product_uav_xz <<  std::endl;
			// std::cout << "norm_vector1_uav_xz= " <<norm_vector1_uav_xz << " , norm_vector2_uav_xz= " << norm_vector2_uav_xz << std::endl;
			// std::cout << "statePos1[0]= " << statePos1[0] << " , statePos2[0]= " << statePos2[0] << " , statePos3[0]= " << statePos3[0] << std::endl;
			// std::cout << "angle_uav_xy= " <<angle_uav_xy << " , angle_uav_xz= " << angle_uav_xz << std::endl;
			// std::cout << "value_xy= " << value_xy << " , value_xz= " << value_xz << std::endl;
			// std::cout << "residual[0]= " << residual[0] << " , residual[1]= " << residual[1] << std::endl;
			

			return true;
		}
  }

 double wf_, ang_;

 private:
};


#endif