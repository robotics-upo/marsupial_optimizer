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
			NumPointParable ():
			{}
			bool operator()(const double *state, int *ret_) const 
			{
				int num_point_per_unit_length = 10;
		
				int num_point_parable = round( (double)num_point_per_unit_length * state);
				ret_[0] = num_point_parable; // in case rayCast with collision
				
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
				double d_obs, cost_;
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
			ParableFunctor(double weight_factor, Grid3d* grid_3D_, geometry_msgs::Vector3 pos_reel_ugv, doble sb, bool write_data, std::string user_name)
					: wf(weight_factor), g_3D_(grid_3D_), pos_reel_ugv_(pos_reel_ugv), sb_(sb), w_d_(write_data), user_(user_name)
			{
				num_point_parable.reset(new ceres::CostFunctionToFunctor<1,1>(
							  new ceres::NumericDiffCostFunction<NumPointParable, ceres::CENTRAL,1,1>(new NumPointParable())));
				cost_parable.reset(new ceres::CostFunctionToFunctor<1,3>(
							  new ceres::NumericDiffCostFunction<NumPointParable, ceres::CENTRAL,1,3>(new NumPointParable(g_3D_, sb_))));
			}

			template <typename T>
			bool operator()(const T* const pUGV, const T* const pUAV, const T* const param, T* residual) const 
			{
				T ugv_reel[4] = {pUGV[0], pUGV[1], pUGV[2], pUGV[3] + T{pos_reel_ugv.z}}; // Set first parable point on the reel position
				T d = sqrt(pow(pUAV[1]-ugv_reel[1],2)+pow(pUAV[2]-ugv_reel[2],2)); // To not make less than num_point_per_unit_length the value of points in parable
				if (d_ < 1.0)
					d_ = T{1.0};

				// Here Compute the Parable Points
				T p_[3]; //  To save parable point to computed distance to obstacle
				T points_in_parable[1];
				(*num_point_parable)(d_, points_in_parable); // To get points number in parable
			
				//**FALTA ARREGLAR ESTO**//
				double x_, tetha_;
				T x_  =  T{0.0};
				checkStateParable(p1_, p2_);
				tetha_ = atan(fabs(p2_.y - p1_.y)/fabs(p2_.x - p1_.x));
				//**FALTA ARREGLAR ESTO**//
			
				T parable_cost_ = T{0.0};
			
				if (!x_const || !y_const){ 
					for(int i = 0; i < points_in_parable; i++){
						x_ = x_ + (d_/(double)points_in_parable);
						p_[0] = pUGV[1] + T{_direc_x* cos(tetha_) * x_};
						p_[1] = pUGV[2] + T{_direc_y* sin(tetha_) * x_};
						p_[2] = param_[1] * x_* x_ + param_[2] * x_ + param_[3];
						(*cost_parable)(p_, parable_cost_); // To get points number in parable
					}
				}
				else{
					double d_ = fabs(p1_.z - p2_.z);
					double _step = d_ / (double) points_in_parable;
					for(int i=0; i < points_in_parable ; i++)
					{       
						p_[0] = pUGV[1];
						p_[1] = pUGV[2];
						p_[2] = pUGV[3]+ T{_step* (double)i};    
						(*cost_parable)(p_, parable_cost_); // To get points number in parable
					}
				}

				residual[0] = wf * stateParable[1];
					
				return true;
			}
		
			std::unique_ptr<ceres::CostFunctionToFunctor<1,1> > num_point_parable;
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