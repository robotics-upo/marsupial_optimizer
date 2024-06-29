#ifndef CERES_CONSTRAINS_PARABOLA_OBSTACLES_AUTODIFF_HPP
#define CERES_CONSTRAINS_PARABOLA_OBSTACLES_AUTODIFF_HPP

#include "ceres/ceres.h"
#include "ceres/cost_function_to_functor.h"

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

class DistanceFunctionObstacles : public ceres::SizedCostFunction<1, 3> 
{
 public:

    DistanceFunctionObstacles(Grid3d* grid_): g_3D(grid_)
    {
    }

    virtual ~DistanceFunctionObstacles(void) 
    {
    }
	
    virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const 
    {
        double x = parameters[0][0];
        double y = parameters[0][1];
        double z = parameters[0][2];
		double distance_;
		double min_dist_ = 0.005;

		if(g_3D->isIntoMap(x, y, z)){
			TrilinearParams p = g_3D->getPointDistInterpolation(x, y, z);
            distance_ = p.a0 + p.a1*x + p.a2*y + p.a3*z + p.a4*x*y + p.a5*x*z + p.a6*y*z + p.a7*x*y*z;

			if(distance_ < min_dist_)
				distance_ = min_dist_;
	
			residuals[0] = distance_;

			if(jacobians != NULL && jacobians[0] != NULL){
				jacobians[0][0] = p.a1 + p.a4*y + p.a5*z + p.a7*y*z;
				jacobians[0][1] = p.a2 + p.a4*x + p.a6*z + p.a7*x*z;
				jacobians[0][2] = p.a3 + p.a5*x + p.a6*y + p.a7*x*y;
			}
		}
        else{
			distance_ = -1.0;
			residuals[0] = distance_;
			if (jacobians != NULL && jacobians[0] != NULL) 
            {
                jacobians[0][0] = 0.0;
                jacobians[0][1] = 0.0;
                jacobians[0][2] = 0.0;
            }
        }

        return true;
  }
  private:
	Grid3d *g_3D;
};

class NumPointParabola : public ceres::SizedCostFunction<1, 1> 
{
 public:

    NumPointParabola(void)
    {
    }

    virtual ~NumPointParabola(void) 
    {
    }

    virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const 
    {
		int num_point_per_unit_length = 20;
		
		residuals[0] = round( (double)num_point_per_unit_length * parameters[0][0] );
        if (jacobians != NULL && jacobians[0] != NULL) 
        	jacobians[0][0] = num_point_per_unit_length;

        return true;
  }
  private:

};

class AutodiffParableFunctor {

	public:
    AutodiffParableFunctor(){}

		struct ParableFunctor 
		{
			ParableFunctor(double weight_factor_, Grid3d* grid_3D_, geometry_msgs::Vector3 pos_reel_ugv_, double sb_, bool write_data_, std::string user_name_)
			: wf(weight_factor_), g_3D(grid_3D_), pos_reel_ugv(pos_reel_ugv_), sb(sb_), w_d_(write_data_), user(user_name_), 
			_parableCostFunctor(new DistanceFunctionObstacles(g_3D)), _numPointFunctor(new NumPointParabola())
			{
			}

			// ParableFunctor(double weight_factor_, Grid3d* grid_3D_, Grid3d* grid_3D_obst_, Grid3d* grid_3D_trav_, 
			// 				geometry_msgs::Vector3 pos_reel_ugv_, double sb_, bool write_data_, std::string user_name_)
			// : wf(weight_factor_), g_3D(grid_3D_), g_3D_obst(grid_3D_obst_), g_3D_trav(grid_3D_trav_), 
			// 			pos_reel_ugv(pos_reel_ugv_), sb(sb_), w_d_(write_data_), user(user_name_), 
			// _parableCostFunctor(new DistanceFunctionObstacles(g_3D, g_3D_obst, g_3D_trav, sb)), _numPointFunctor(new NumPointParabola())
			// {
			// }

			template <typename T>
			bool operator()(const T* const pUGV, const T* const pUAV, const T* const param, T* residual) const 
			{
				T ugv_reel[4] = {pUGV[0], pUGV[1], pUGV[2], pUGV[3] + T{pos_reel_ugv.z}}; // Set first parable point on the reel position
				
				// Here is compute the Parable Parameters 
				double min_val_ = 0.01;
				double min_dist_ = 0.005;
				T d_[1];
				T np_, u_x , u_y;
				bool x_const, y_const;
				x_const = y_const = false;
				vector<int> v_coll;

				T delta_x = (pUAV[1] - ugv_reel[1]);
				T delta_y = (pUAV[2] - ugv_reel[2]);
				T delta_z = (pUAV[3] - ugv_reel[3]);
				T dist_ = sqrt(delta_x * delta_x + delta_y * delta_y );
				if (dist_ < min_val_){
					u_x = u_y = T{0.0};
					d_[0] = sqrt(delta_z*delta_z); 
				} else{
					u_x = delta_x /dist_;
					u_y = delta_y /dist_;
					d_[0] = sqrt(delta_x * delta_x + delta_y * delta_y + delta_z*delta_z);
				}
				if (d_[0] < 1.0) // To not make less than num_point_per_unit_length the value of points in parable
					d_[0] = T{1.0};

        		_numPointFunctor(d_, &np_); // To get the values and parameters needed for computing the parable interpolation

				// Here is compute the parable point and it cost 
				T point[3];
				T dist_to_obst_;	
				T x_  =  T{0.0};
				vector<T> v_dist ; v_dist.clear();
				vector<T> v_x_, v_y_, v_z_; v_x_.clear(); v_y_.clear(); v_z_.clear();
				vector<T> vx_coll_, vy_coll_, vz_coll_; vx_coll_.clear(); vy_coll_.clear(); vz_coll_.clear();

				vector<int> v_transition_coll_; v_transition_coll_.clear();
				int first_coll_, last_coll_, prev_coll_;
				first_coll_ = last_coll_ = prev_coll_ = -2;
				for(int i = 0; i < np_; i++, x_ += dist_/ np_ ){  
					if (!(dist_ < min_val_)){ // To check difference position between UGV and UAV only in z-axe, so parable it is not computed
						point[0] = ugv_reel[1] + u_x * x_;
						point[1] = ugv_reel[2] + u_y * x_;
						point[2] = param[1] * x_* x_ + param[2] * x_ + param[3];
					}
					else{ 	// In case that UGV and UAV are not in the same point the plane X-Y
						T _step = d_[0] / np_;
						point[0] = ugv_reel[1];
						point[1] = ugv_reel[2];
						point[2] = ugv_reel[3]+ _step* (double)i;    	
					}
        			_parableCostFunctor(point, &dist_to_obst_);
					// Data save for aware of the tether status
					v_dist.push_back(dist_to_obst_);
					v_x_.push_back(point[0]);
					v_y_.push_back(point[1]);
					v_z_.push_back(point[2]);
					// Clasification of collision with floor or other obstacles
					if(dist_to_obst_ < T(0.0) || pUGV[3] >= point[2])
						v_coll.push_back(-1);
					else if (dist_to_obst_ < T(sb))
						v_coll.push_back(1);
					else
						v_coll.push_back(0);

					// Algorithm to check intervals points in collision
					if(v_coll[i] == 1 && prev_coll_ == 0){
						first_coll_ = i;
						prev_coll_ = 1;
					}else if( v_coll[i] == 0 && prev_coll_ == 1 && first_coll_!=2){
						last_coll_ = i-1;
						prev_coll_ = 0;
					}else if (v_coll[i] == 1){
						prev_coll_ = 1;
					}else if (v_coll[i] == 0){
						prev_coll_ = 0;
					}else if (v_coll[i] == -1){
						prev_coll_ = -1;
						first_coll_ = last_coll_ = -2;
					}
					if (first_coll_ != -2 && last_coll_ != -2){
						v_transition_coll_.push_back(first_coll_);
						v_transition_coll_.push_back(last_coll_);
						first_coll_ = last_coll_ = -2;
						vx_coll_.push_back(v_x_[first_coll_]);
						vy_coll_.push_back(v_y_[first_coll_]);
						vz_coll_.push_back(v_z_[first_coll_]);
						vx_coll_.push_back(v_x_[last_coll_]);
						vy_coll_.push_back(v_y_[last_coll_]);
						vz_coll_.push_back(v_z_[last_coll_]);
					}
				}
				int group_coll_, first_coll , last_coll , middle_coll;
				group_coll_ = first_coll = last_coll = middle_coll = -1;
				if(v_transition_coll_.size() > 0){
					group_coll_ = (v_transition_coll_.size()/2);
					first_coll = v_transition_coll_[0];
					last_coll = v_transition_coll_[v_transition_coll_.size()-1];
					middle_coll = ceil((last_coll-first_coll)/2);
				}

				T distance_, C, point_cost_, cost_;
				T cost_state_parable = T{0.0};
				for(int i = 0; i < np_; i++){ 
					if (v_coll[i]== -1){
						C = T{10.0};
						distance_ = 100.0*(pUGV[3] - v_z_[i]);
						double y_offset = -1 + 1/min_dist_;
						cost_ = (exp(distance_) + T{y_offset});
					}
					else if (i >= first_coll && i <= last_coll && v_transition_coll_.size()> 0){
						if (v_dist[i] < T{sb} && v_dist[i] > T{0.01}){
							C = T{40.0};
							distance_ = v_dist[i];
							cost_ = T{1.0}/distance_;
						}else{
							C = T{1.0};
							distance_ = T{-3.0};
							cost_ = T{2000.0};	
						}
					}
					else if (v_coll[i]== 0){
						C = T{1.0};
						distance_ = v_dist[i];
						cost_ = T{1.0}/distance_;			
					}

					point_cost_ = cost_*C;
					// std::cout << "["<<i<<"]cost:" << point_cost_ <<" , d:" << distance_ <<" , C:" << C << " ,v_coll[i]=" << v_coll[i]<<std::endl;
					// std::cout << "["<<i<<"]  d:" << distance_ <<" , C:" << C << " ,v_coll=" << v_coll[i]<<std::endl;
					// std::cout <<"	Point:["<< v_x_[i]<<","<< v_y_[i]<< ","<< v_z_[i]<<"]"<<std::endl;
					cost_state_parable = cost_state_parable + point_cost_; // To get point parable cost
				}
				double num_coll_ = 0.0;
				// if(v_transition_coll_.size() > 0){
				// 	num_coll_ = double(v_transition_coll_[v_transition_coll_.size()-1]- v_transition_coll_[0] );
				// 	for(size_t i = 0; i < group_coll_; i++){
				// 		std::cout << "intervall collision positions:["<< v_transition_coll_[2*i]<< " " << v_transition_coll_[2*i+1] <<"] v_transition_coll_.size()="<< v_transition_coll_.size() << std::endl;
				// 	}
				// }
				// cost_state_parable = cost_state_parable + T{num_coll_}*T{1000.0};
			// if(v_transition_coll_.size() > 0)
			// 	std::cout << "collisions:["<< v_transition_coll_[0]  << "," << v_transition_coll_[v_transition_coll_.size()-1] <<"] num_coll_:[" << num_coll_ << "]" << std::endl;
			// std::cout << "group_coll_:"<< group_coll_<< "  v_transition_coll_.size:"<< v_transition_coll_.size() <<std::endl;
				cost_state_parable = cost_state_parable/np_;

				residual[0] = wf * cost_state_parable;
// std::cout << "	Obs["<< param[0]<<"] : R[0]:"  << residual[0] << " P:["<< param[1]<<"/ "  << param[2] <<"/ "  << param[3] <<"]"<< std::endl ;
// std::cout << "Obs["<< param[0]<<"] : R[0]:"  << residual[0] << std::endl ;
				return true;
			}

			bool w_d_;
			double wf, sb;
			geometry_msgs::Vector3 pos_reel_ugv;
			std::string user;
			// Grid3d* g_3D;
			Grid3d *g_3D, *g_3D_obst, *g_3D_trav;
	    	ceres::CostFunctionToFunctor<1, 3> _parableCostFunctor;
	    	ceres::CostFunctionToFunctor<1, 1> _numPointFunctor;
		};
	private:
};

#endif