#ifndef CERES_CONSTRAINS_PARABLE_OBSTACLES_AUTODIFF_HPP
#define CERES_CONSTRAINS_PARABLE_OBSTACLES_AUTODIFF_HPP

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
using ceres::NumericDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;


class AutodiffParableFunctor {

public:
    AutodiffParableFunctor(){}

	struct ParableFunctor 
	{
	ParableFunctor(double weight_factor, Grid3d* grid_3D_, geometry_msgs::Vector3 pos_reel_ugv, ros::NodeHandlePtr nhP_, bool write_data, std::string user_name)
					: wf(weight_factor), g_3D_(grid_3D_), pos_reel_ugv_(pos_reel_ugv), w_d_(write_data), user_(user_name)
		{
			nhP = nhP_;
		}

		template <typename T>
		bool operator()(const T* const stateUGV, const T* const stateUAV, const T* const stateParable, T* residual) const 
		{
			T xA[1], xB[1]; 
			T yA[1] = {stateUGV[3] + T{pos_reel_ugv_.z}};
			T yB[1] = {stateUAV[3]};
			T dX[1] = {sqrt(pow(stateUAV[1] - stateUGV[1],2) + pow(stateUAV[2] - stateUGV[2],2))};
			T dY[1] = {stateUAV[3] - stateUGV[3]};
			T cross_P[3];

			//Compute Direction Vector for plane
			// T vd[3] = {stateUAV[0]-stateUGV[0], stateUAV[1]-stateUGV[1], stateUAV[2]-stateUGV[2]};
			// T va[3] = {stateUAV[0]-stateUGV[0] + T{1.0}, stateUAV[1]-stateUGV[1], stateUAV[2]-stateUGV[2]}}
			//Compute cross product
			// cross_P[0] = vd[1] * va[2] - vd[2] * va[1];
			// cross_P[1] = vd[2] * va[0] - vd[0] * va[2];
			// cross_P[2] = vd[0] * va[1] - vd[1] * va[0];

			// residual[0] = wf * (stateParable[1]* xA*xA + stateParable[2]*xA + stateParable[3] - yA);
			// residual[1] = wf * (stateParable[1]* xB*xB + stateParable[2]*xB + stateParable[3] - yB);
			residual[0] = wf * stateParable[1];
			residual[1] = wf * stateParable[1];

					
		return true;
		}
		
		bool w_d_;
		double wf;
		geometry_msgs::Vector3 pos_reel_ugv_;
		std::string user_;

		Grid3d* g_3D_;
		ros::NodeHandlePtr nhP;
	};

private:

};

#endif