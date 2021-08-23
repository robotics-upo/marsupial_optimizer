#ifndef CERES_CONSTRAINS_LENGTH_UGV_HPP
#define CERES_CONSTRAINS_LENGTH_UGV_HPP


#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"
#include <ros/ros.h>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

class LengthFunctorUGV 
{

public:
  	LengthFunctorUGV(double weight_factor, double dist_1, double dist_2, double dist_3, double dist_4, double dist_5): 
	  				 wf_(weight_factor), d1_(dist_1), d2_(dist_2), d3_(dist_3), d4_(dist_4), d5_(dist_5) {}

  	template <typename T>
  	bool operator()(const T* const statePos1, 
	  				const T* const statePos2, 
	  				const T* const statePos3, 
	  				const T* const statePos4, 
	  				const T* const statePos5, 
					const T* const statePos6, T* residual) const 
	{
		// Length UGV trajectory
		T arg_dist_1_ = pow(statePos2[1]-statePos1[1],2) + pow(statePos2[2]-statePos1[2],2) + pow(statePos2[3]-statePos1[3],2);
		T arg_dist_2_ = pow(statePos3[1]-statePos2[1],2) + pow(statePos3[2]-statePos2[2],2) + pow(statePos3[3]-statePos2[3],2);
		T arg_dist_3_ = pow(statePos4[1]-statePos3[1],2) + pow(statePos4[2]-statePos3[2],2) + pow(statePos4[3]-statePos3[3],2);
		T arg_dist_4_ = pow(statePos5[1]-statePos4[1],2) + pow(statePos5[2]-statePos4[2],2) + pow(statePos5[3]-statePos4[3],2);
		T arg_dist_5_ = pow(statePos6[1]-statePos5[1],2) + pow(statePos6[2]-statePos5[2],2) + pow(statePos6[3]-statePos5[3],2);

		T dist_1_, dist_2_, dist_3_, dist_4_, dist_5_;
		if (arg_dist_1_ > 0.0001)
			dist_1_ = sqrt(arg_dist_1_);
		else
			dist_1_ = T{0.0};
		if (arg_dist_2_ > 0.0001)
			dist_2_ = sqrt(arg_dist_2_);
		else
			dist_2_ = T{0.0};
		if (arg_dist_3_ > 0.0001)
			dist_3_ = sqrt(arg_dist_3_);
		else
			dist_3_ = T{0.0};
		if (arg_dist_4_ > 0.0001)
			dist_4_ = sqrt(arg_dist_4_);
		else
			dist_4_ = T{0.0};
		if (arg_dist_5_ > 0.0001)
			dist_5_ = sqrt(arg_dist_5_);
		else
			dist_5_ = T{0.0};
		
		T dist_ = dist_1_ + dist_2_ + dist_3_ + dist_4_ + dist_5_;
	
		// Reference Length initial path
		double d_ref_ = d1_ + d2_ + d3_ + d4_ + d5_;

		//Compute Residual
		if (dist_ < d_ref_)
			residual[0] =  T{0.0};
		else
			residual[0] =  wf_ * 100.0 * (exp(dist_ - d_ref_)-1.0);
		

		return true;
		
	}

	double wf_, d1_, d2_, d3_, d4_, d5_;

private:

};


#endif