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
		
		
			
		if (dot_product_ugv < 0.0001 && dot_product_ugv > -0.0001 ){
			// printf("\nKinematicsFunctorUGV if, ");
			// std::cout << "statePos1[0]= " << statePos1[0] << " , statePos2[0]= " << statePos2[0] << " , statePos3[0]= " << statePos3[0] << std::endl;
			// std::cout << "dot_product_ugv= " << dot_product_ugv << std::endl;
			residual[0] = T(0.0);
			return true;
		}
		else{
			// Calculate norm 
			arg_norm_vector1_ugv = (vector1_ugv[0] * vector1_ugv[0]) + (vector1_ugv[1] * vector1_ugv[1]);
			if(arg_norm_vector1_ugv < 0.001 && arg_norm_vector1_ugv > -0.001)
				norm_vector1_ugv = T{0.0};
			else
				norm_vector1_ugv = sqrt(arg_norm_vector1_ugv);
			// Calculate norm 
			arg_norm_vector2_ugv = (vector2_ugv[0] * vector2_ugv[0]) + (vector2_ugv[1] * vector2_ugv[1]);
			if(arg_norm_vector2_ugv < 0.001 && arg_norm_vector2_ugv > -0.001)
				norm_vector2_ugv = T{0.0};
			else
				norm_vector2_ugv = sqrt(arg_norm_vector2_ugv);
			// Define angle threshold
			bound1_ugv = T(-ang_);
			bound2_ugv = T(ang_);

			T dist_12 , dist_13;
			T arg_dist_12, arg_dist_13;
			T value;

			// Compute distance between positions
			arg_dist_12 = (statePos2[1]-statePos1[1])*(statePos2[1]-statePos1[1]) + 
						  (statePos2[2]-statePos1[2])*(statePos2[2]-statePos1[2]) + 
						  (statePos2[3]-statePos1[3])*(statePos2[3]-statePos1[3]) ;
			if(arg_dist_12 < 0.001 && arg_dist_12 > -0.001)
				dist_12 = T{0.0};
			else
			dist_12 = sqrt(arg_dist_12);
			// Compute distance between positions
			arg_dist_13 = (statePos3[1]-statePos2[1])*(statePos3[1]-statePos2[1]) + 
						  (statePos3[2]-statePos2[2])*(statePos3[2]-statePos2[2]) + 
						  (statePos3[3]-statePos2[3])*(statePos3[3]-statePos2[3]) ;
			if(arg_dist_13 < 0.001 && arg_dist_13 > -0.001)
				dist_13 = T{0.0};
			else
				dist_13 = sqrt(arg_dist_13);

			T arg_acos;
			if((norm_vector1_ugv < 0.001 && norm_vector1_ugv > -0.001) || (norm_vector2_ugv < 0.001 && norm_vector2_ugv > -0.001)){
				arg_acos = T{0.0};	
			}else {
				arg_acos = dot_product_ugv / (norm_vector1_ugv*norm_vector2_ugv);
			}

			// T arg_acos = dot_product_ugv / (norm_vector1_ugv*norm_vector2_ugv);
			if (dist_12 < 0.001 || dist_13 < 0.001 || arg_acos > 1.000 || arg_acos < -1.000)
				angle_ugv = T{0.0};
			else
				angle_ugv = acos(arg_acos);

			if (angle_ugv < 0.001 && angle_ugv > -0.001)
				value = T{0.0};
			else
				value = sqrt(angle_ugv*angle_ugv);

			// To compute residual 	
			if ( (angle_ugv < bound1_ugv) || (angle_ugv > bound2_ugv) ) 
				residual[0] =  wf_ * exp(2.0* value );
			else
				residual[0] = T(0.0);

	        // printf("\nKinematicsFunctorUGV else, ");
			// std::cout << "statePos1[0]= " << statePos1[0] << " , statePos2[0]= " << statePos2[0] << " , statePos3[0]= " << statePos3[0] << std::endl;
			// // std::cout << "statePos1[1]= " << statePos1[1] << " , statePos1[2]= " << statePos1[2] << std::endl;
			// // std::cout << "statePos2[1]= " << statePos2[1] << " , statePos2[2]= " << statePos2[2] << std::endl;
			// // std::cout << "statePos3[1]= " << statePos3[1] << " , statePos3[2]= " << statePos3[2] << std::endl;
			// std::cout << "dot_product_ugv= " << dot_product_ugv << " , norm_vector1_ugv= " << norm_vector1_ugv << " , norm_vector2_ugv= " << norm_vector2_ugv<< std::endl;
			// std::cout << "dist_12= " << dist_12 << " , dist_13= " << dist_13 << " , arg_acos= " << arg_acos<< std::endl;
			// // std::cout << "value= " << value << " , angle_ugv= " << angle_ugv << std::endl;
			// std::cout << "bound1_ugv= " << bound1_ugv << " , bound2_ugv= " << bound2_ugv << std::endl;
			// std::cout << "residual[0]= " << residual[0] << " , angle_ugv= " << angle_ugv << " , value= " << value << std::endl;

			return true;
		}
	}

	double wf_, ang_;

private:

};


#endif