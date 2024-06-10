#ifndef CERES_CONSTRAINS_STRAIGHT_LENGTH_AUTODIFF_HPP
#define CERES_CONSTRAINS_STRAIGHT_LENGTH_AUTODIFF_HPP

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

#include <iostream>
#include <fstream>
#include <string>

using ceres::AutoDiffCostFunction;
using ceres::NumericDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

class AutodiffStraightLengthFunctor {

public:
    AutodiffStraightLengthFunctor(){}

	struct StraightLengthFunctor 
	{
	StraightLengthFunctor(double weight_factor, geometry_msgs::Vector3 pos_reel_ugv_)
					: wf(weight_factor), pos_reel_ugv(pos_reel_ugv_)
		{}

		template <typename T>
		bool operator()(const T* const pUGV, const T* const pUAV, const T* const length, T* residual) const 
		{
			T ugv_reel[4] = {pUGV[0], pUGV[1], pUGV[2], pUGV[3] + T{pos_reel_ugv.z}}; // Set first parable point on the reel position
			T dist = T{1.001} * sqrt(pow(pUAV[1]-ugv_reel[1],2)+pow(pUAV[2]-ugv_reel[2],2)+pow(pUAV[3]-ugv_reel[3],2)); 
			
			T L = length[1];

			T diff_;
			diff_ = (dist - L);

			residual[0] = wf * 1.0* (exp(diff_)-1.0) ;
					
			return true;
		}
		
		double wf;
		geometry_msgs::Vector3 pos_reel_ugv;
	};

private:

};

#endif