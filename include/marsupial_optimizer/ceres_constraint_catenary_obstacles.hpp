#ifndef CERES_CONSTRAINS_CATENARY_OBSTACLES_AUTODIFF_HPP
#define CERES_CONSTRAINS_CATENARY_OBSTACLES_AUTODIFF_HPP

#include "ceres/ceres.h"
#include "ceres/cost_function_to_functor.h"

#include "glog/logging.h"
#include "Eigen/Core"

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
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

class DistanceFunction : public ceres::SizedCostFunction<1, 3> 
{
 public:

    DistanceFunction(Grid3d* grid_)
      : g_3D(grid_)
    {
    }

    virtual ~DistanceFunction(void) 
    {
    }
	
    virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const 
    {
        float x = parameters[0][0];
        float y = parameters[0][1];
        float z = parameters[0][2];
		float dist_;
		double min_dist_ = 0.005;

		if(g_3D->isIntoMap(x, y, z))
        {
			TrilinearParams p = g_3D->getPointDistInterpolation(x, y, z);
            dist_ = p.a0 + p.a1*x + p.a2*y + p.a3*z + p.a4*x*y + p.a5*x*z + p.a6*y*z + p.a7*x*y*z;

			if(dist_ < min_dist_)
				dist_ = min_dist_;

			residuals[0] = dist_;
            if (jacobians != NULL && jacobians[0] != NULL) 
            {
				jacobians[0][0] = p.a1 + p.a4*y + p.a5*z + p.a7*y*z;
                jacobians[0][1] = p.a2 + p.a4*x + p.a6*z + p.a7*x*z;
                jacobians[0][2] = p.a3 + p.a5*x + p.a6*y + p.a7*x*y;
            }
        }
        else
        {
			dist_ = -1.0;
            if (jacobians != NULL && jacobians[0] != NULL) 
            {
                jacobians[0][0] = 0;
                jacobians[0][1] = 0;
                jacobians[0][2] = 0;
            }
        }

        return true;
  }
  private:
	Grid3d* g_3D;
};

class NumPointFunction : public ceres::SizedCostFunction<1, 1> 
{
 public:
    NumPointFunction(void)
    {
    }
    virtual ~NumPointFunction(void) 
    {
    }
    virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const 
    {
		int num_point_per_unit_length = 10;
		residuals[0] = round( (double)num_point_per_unit_length * parameters[0][0] );
        if (jacobians != NULL && jacobians[0] != NULL) 
        	jacobians[0][0] = num_point_per_unit_length;
        return true;
  }
  private:
};

class AutodiffCatenaryObstacleFunctor {

	public:
    AutodiffCatenaryObstacleFunctor(){}

		struct CatenaryObstacleFunctor 
		{
			CatenaryObstacleFunctor(double weight_factor_, Grid3d* grid_3D_, geometry_msgs::Point pos_reel_ugv_, double sb_, bool write_data_, std::string user_name_)
			: wf(weight_factor_), g_3D(grid_3D_), pos_reel_ugv(pos_reel_ugv_), sb(sb_), w_d_(write_data_), user(user_name_), 
			_parableCostFunctor(new DistanceFunction(g_3D)), _numPointFunctor(new NumPointFunction())
			{
			}

			template <typename T>
			bool operator()(const T* const stateUGV, const T* const stateUAV, const T* const params, T* residual) const 
			{
				T ugv_reel[4] = {stateUGV[0], stateUGV[1], stateUGV[2], stateUGV[3] + T{pos_reel_ugv.z}}; // Set first parable point on the reel position
				
				// Here is compute the Parable Parameters 
				double min_val_ = 0.01;
				T d_[1];
				T np_, u_x , u_y;
				bool x_const, y_const;
				x_const = y_const = false;
				vector<int> v_coll;

				/* Map:
				params[1] = Xo 
				params[2] = Yo 
				params[3] = a 
				params[4] = length (Not used length because it is not optimized)
				*/
				T delta_x = (stateUAV[1] - ugv_reel[1]);
				T delta_y = (stateUAV[2] - ugv_reel[2]);
				T delta_z = (stateUAV[3] - ugv_reel[3]);
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
					point[2] = params[3] * cosh((x_ - params[1])/params[3]) +( params[2]-params[3]);
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
					// Tether point collision clasification with floor or other obstacles
					if(dist_to_obst_ < T(0.0) || point[2] < stateUGV[3]+T{sb}) // floor collision
						v_coll.push_back(-1);
					else if (dist_to_obst_ < T{sb}) // other obstacles collision
						v_coll.push_back(1);
					else
						v_coll.push_back(0);  // not collision

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
					if (first_coll_ != -2 && last_coll_ != -2){ // Points in transition that are detected as free collision in a tether collision position are identify 
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
				T m1_, m2_, Cmax;
				group_coll_ = first_coll = last_coll = middle_coll = -1;
				if(v_transition_coll_.size() > 0){ // this vector only is fill when the are tether points far from collision (v_coll = 0), even though tether is in collision
					group_coll_ = (v_transition_coll_.size()/2);
					first_coll = v_transition_coll_[0];
					last_coll = v_transition_coll_[v_transition_coll_.size()-1];
					middle_coll = ceil((last_coll+first_coll)/2);
					double count_;
					for(int i = 0; i < np_; i++){ 
						if (i >= first_coll && i <= last_coll && v_coll[i]== 0 )
							count_++;
					}
					double max_cost_ = 200.0 + 20.0 * count_;
					Cmax = T{max_cost_};
					T min_cost1_ = T{1.0}/ v_dist[first_coll];
					T min_cost2_ = T{1.0}/ v_dist[last_coll];
					double diff1_ = double(middle_coll- first_coll);
					double diff2_ = double(last_coll- middle_coll);
						if (diff1_ ==0)
							diff1_ = 1;
						if (diff2_ ==0)
							diff2_ = 1;
					m1_ = ( (Cmax - min_cost1_)/ T{diff1_} );
					m2_ = ( (min_cost2_ - Cmax)/ T{diff2_} );
				}

				T distance_, C, point_cost_, cost_;
				T cost_state_parable = T{0.0};
				for(int i = 0; i < np_; i++){ 
					if (v_coll[i]== -1){
						if (v_dist[i] > 0){
							C = T{10.0};
							distance_ = v_dist[i];
							cost_ = T{1.0}/distance_;
						}
						else{
							C = T{1.0};
							distance_ = T{-3.0};
							cost_ = T{10000.0};	
						}
					}else if (v_coll[i]== 1){
						if (i >= first_coll && i <= middle_coll && v_transition_coll_.size() > 0){
							C = T{20.0};
							double diff_x_ = double(i - first_coll);
							distance_ = T{diff_x_};
							cost_ =  m1_ * T{diff_x_} + T{1.0}/ v_dist[first_coll];
						}else if (i > middle_coll && i <= last_coll && v_transition_coll_.size() > 0){
							C = T{20.0};
							double diff_x_ = double(i - middle_coll);
							distance_ = T{diff_x_};
							cost_ =  m2_ * T{diff_x_} + Cmax;
						}
						else{
							C = T{10.0};
							distance_ = v_dist[i];
							cost_ = T{1.0}/distance_;
						}
					}
					else{
						C = T{1.0};
						distance_ = v_dist[i];
						cost_ = T{1.0}/distance_;			
					}
					point_cost_ = cost_*C;
					cost_state_parable = cost_state_parable + point_cost_; // To get point parable cost
				}

				double num_coll_ = 0.0;
				cost_state_parable = cost_state_parable/np_;

				residual[0] = wf * cost_state_parable;
// std::cout << "	Obs["<< params[0]<<"] : R[0]:"  << residual[0] << std::endl ;
				return true;
			}

			bool w_d_;
			double wf, sb;
			geometry_msgs::Point pos_reel_ugv;
			std::string user;
			Grid3d* g_3D;
	    	ceres::CostFunctionToFunctor<1, 3> _parableCostFunctor;
	    	ceres::CostFunctionToFunctor<1, 1> _numPointFunctor;
		};
	private:
};

#endif