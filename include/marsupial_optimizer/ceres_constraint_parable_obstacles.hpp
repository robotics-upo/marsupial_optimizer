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

		struct NumPointParable 
		{
			NumPointParable()
			{}
			// bool operator()(const double *dist, const double *p1, const double *p2, const double *param, double *point) const 
			bool operator()(const double *dist, const double *p1, const double *p2, const double *param, double *np) const 
			{
				int num_point_per_unit_length = 10;
				int num_point_parable = round( (double)num_point_per_unit_length * dist[0]);

				int u_x, u_y;

				if(p2[1] > p1[1])
					u_x = 1;
				else if(p2[1] < p1[1])
					u_x = -1;
				else
					u_x = 0;    
				
				if(p2[2] > p1[2])
					u_y = 1;
				else if(p2[2] < p1[2])
					u_y = -1;
				else
					u_y = 0;  
				
				int change_x = 1;
				int change_y = 1;
				
				if (fabs(p1[1] - p2[1]) < 0.001) 
					change_x = -1;
				
				if (fabs(p1[2] - p2[2]) < 0.001)
					change_y = -1;

				np[0] = num_point_parable;
				np[1] = u_x;
				np[2] = u_y;
				np[3] = change_x;
				np[4] = change_y;

				return true;
			}
		};

		struct CostDistanceObstacle 
		{
			CostDistanceObstacle (Grid3d* g_3D_, double sb_): g_3D(g_3D_), sb(sb_)
			{}
			bool operator()(const double *state, double *distance_cost) const 
			{
				bool is_into_ = g_3D->isIntoMap((double)state[0],(double)state[1],(double)state[2]);
				double d_obs, cost_, x_, y_, z_;
				if (is_into_){
					TrilinearParams d = g_3D->getPointDistInterpolation((double)state[0], (double)state[1], (double)state[2]);
					x_ = state[0];
					y_ = state[1];
					z_ = state[2];
					d_obs= (d.a0 + d.a1*x_ + d.a2*y_ + d.a3*z_ + d.a4*x_*y_ + d.a5*x_*z_ + d.a6*y_*z_ + d.a7*x_*y_*z_);
				}
				else
					d_obs = sb;
				
				if (d_obs < sb)
					cost_ = (1.0/d_obs)*2.0;
				else
					cost_ = (1.0/d_obs)*1.0;		

				distance_cost[0] = cost_;
				return true;	
			}
			double sb;
			Grid3d* g_3D;
		};

		struct ParableFunctor 
		{
			ParableFunctor(double weight_factor, Grid3d* grid_3D_, geometry_msgs::Vector3 pos_reel_ugv, double sb, bool write_data, std::string user_name)
					: wf(weight_factor), g_3D_(grid_3D_), pos_reel_ugv_(pos_reel_ugv), sb_(sb), w_d_(write_data), user_(user_name)
			{
				point_parable.reset(new ceres::CostFunctionToFunctor<5,2,4,4,4>(
							  new ceres::NumericDiffCostFunction<NumPointParable, ceres::CENTRAL,5,2,4,4,4>(new NumPointParable())));
				cost_parable.reset(new ceres::CostFunctionToFunctor<1,3>(
							  new ceres::NumericDiffCostFunction<CostDistanceObstacle, ceres::CENTRAL,1,3>(new CostDistanceObstacle(g_3D_, sb_))));
			}

			template <typename T>
			bool operator()(const T* const pUGV, const T* const pUAV, const T* const param, T* residual) const 
			{
				T ugv_reel[4] = {pUGV[0], pUGV[1], pUGV[2], pUGV[3] + T{pos_reel_ugv_.z}}; // Set first parable point on the reel position
				
				T d_[2];
				d_[0] = sqrt(pow(pUAV[1]-ugv_reel[1],2)+pow(pUAV[2]-ugv_reel[2],2)); // To not make less than num_point_per_unit_length the value of points in parable
				if (d_[0] < 1.0)
					d_[0] = T{1.0};
				d_[1] = sqrt(pow(pUAV[3]-ugv_reel[3],2)); 

				// Here Compute the Parable Points
				T np[5]; //  To save parable values anda parameter for parable
				(*point_parable)(d_, pUGV, pUAV, param, np); // To get the values and parameters needed for computing the parable interpolation

				T point[3];	
				T parable_cost_[1];
				T cost_state_parable = T{0.0};
				T x_  =  T{0.0};
				T tetha_ = atan((pUAV[2] - pUGV[2])/(pUAV[1] - pUGV[1]));
				if ( np[3] < 0 && np[4] < 0 ){ // To check position difference between UGV and UAV only in  z-axe, so parable it is not computed
					T _step = d_[0] / T{np[0]};
					for(int i=0; i < np[0] ; i++)
					{       
						point[0] = pUGV[1];
						point[1] = pUGV[2];
						point[2] = pUGV[3]+ _step* (double)i;    
						cost_state_parable = cost_state_parable + parable_cost_[0]; // To get point parable cost
					}
				}
				else{
					for(int i = 0; i < np[0];  i++){
						x_ = x_ + (d_[0]/ T{np[0]});
						point[0] = pUGV[1] + np[1] * cos(tetha_) * x_;
						point[1] = pUGV[2] + np[2] * sin(tetha_) * x_;
						point[2] = param[1] * x_* x_ + param[2] * x_ + param[3];
						(*cost_parable)(point, parable_cost_); // To get point parable cost
						cost_state_parable = cost_state_parable + parable_cost_[0];
					}
				}
				residual[0] = wf * cost_state_parable;
					
				return true;
			}
			std::unique_ptr<ceres::CostFunctionToFunctor<5,2,4,4,4> > point_parable;
			std::unique_ptr<ceres::CostFunctionToFunctor<1,3> > cost_parable;
			bool w_d_, sb_;
			double wf;
			geometry_msgs::Vector3 pos_reel_ugv_;
			std::string user_;
			Grid3d* g_3D_;
		};
private:
};

#endif