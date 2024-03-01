#ifndef CERES_CONSTRAINS_CATENARY_LENGTH_HPP
#define CERES_CONSTRAINS_CATENARY_LENGTH_HPP

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include "catenary_checker/near_neighbor.hpp"
#include "catenary_checker/bisection_catenary_3D.h"
#include "misc/marker_publisher.h"

#include <octomap_msgs/Octomap.h> 
#include <octomap/OcTree.h>

#include <iostream>
#include <fstream>
#include <string>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

class AutodiffTetherLengthFunctor {

public:
	AutodiffTetherLengthFunctor(){}

	struct TetherLengthFunctor 
	{
		TetherLengthFunctor(double weight_factor, geometry_msgs::Vector3 pos_reel_ugv_,  float max_L_, bool write_data, std::string user_name)
					: wf(weight_factor), max_L(max_L_), pos_reel_ugv(pos_reel_ugv_), w_d_(write_data), user_(user_name) 
		{}

		template <typename T>
		bool operator()(const T* const stateUGV, const T* const stateUAV, const T* const params, T* residual) const 
		{
			T ugv_reel[4] = {stateUGV[0], stateUGV[1], stateUGV[2], stateUGV[3] + T{pos_reel_ugv.z}}; // Set first parable point on the reel position
		
			T Xa = T{0.0};
			T Ya = T{0.0};
			T Xb = sqrt(pow(stateUAV[1]-ugv_reel[1],2)+pow(stateUAV[2]-ugv_reel[2],2)); 
			T Yb = stateUAV[3] - stateUGV[3];
			T maxL = T{max_L};
			
			/* Map:
			params[1] = Xo 
			params[2] = Yo 
			params[3] = a 
			params[4] = length (Not used length because it is not optimized)
			*/
			T length = params[3] * sinh((Xa - params[1])/params[3]) + params[3] * sinh((Xb - params[1])/params[3]);
			
			T dist = T{1.005} * sqrt(pow(stateUAV[1]-ugv_reel[1],2)+pow(stateUAV[2]-ugv_reel[2],2)+pow(stateUAV[3]-ugv_reel[3],2)); 

			T diff_;
			if (length < dist )
				diff_ = (dist - maxL);
			else if (length > maxL)
				diff_ = (length - maxL);
			else
				diff_ = T{0.0};

			residual[0] = wf * 100* (exp(diff_)-1.0) ;
					
			return true;
		}
		bool w_d_;
		double wf;
		float max_L;
		geometry_msgs::Vector3 pos_reel_ugv;
		std::string user_;
	};

private:
};

#endif