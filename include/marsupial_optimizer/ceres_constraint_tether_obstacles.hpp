#ifndef CERES_CONSTRAINS_PARABLE_OBSTACLES_AUTODIFF_HPP
#define CERES_CONSTRAINS_PARABLE_OBSTACLES_AUTODIFF_HPP

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

class DistanceFunction : public ceres::SizedCostFunction<1, 3> 
{
 public:

    DistanceFunction(Grid3d* grid_, double security_dist_)
      : g_3D(grid_), sb(security_dist_)
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
		float dist_, div;
		float C;
std::cout << "  TEHER point:"<< x <<"," << y << "," << z <<std::endl;  

		if(g_3D->isIntoMap(x, y, z))
        {
			TrilinearParams p = g_3D->getPointDistInterpolation(x, y, z);
            dist_ = p.a0 + p.a1*x + p.a2*y + p.a3*z + p.a4*x*y + p.a5*x*z + p.a6*y*z + p.a7*x*y*z;
			// div = 1/dist_;
			// if (dist_ < sb)
			// 	C = 10.0;
			// else
			// 	C = 1.0;	
			// residuals[0] = div*C;
			residuals[0] = dist_;
            if (jacobians != NULL && jacobians[0] != NULL) 
            {
                // jacobians[0][0] = -C*(p.a1 + p.a4*y + p.a5*z + p.a7*y*z)*div*div;
                // jacobians[0][1] = -C*(p.a2 + p.a4*x + p.a6*z + p.a7*x*z)*div*div;
                // jacobians[0][2] = -C*(p.a3 + p.a5*x + p.a6*y + p.a7*x*y)*div*div;
				jacobians[0][0] = p.a1 + p.a4*y + p.a5*z + p.a7*y*z;
                jacobians[0][1] = p.a2 + p.a4*x + p.a6*z + p.a7*x*z;
                jacobians[0][2] = p.a3 + p.a5*x + p.a6*y + p.a7*x*y;
            }
// std::cout << " , Cost residual (in) ="<< residuals[0] << " , div= "<< div <<" , dist= " << dist_<< " , C= " << C;
			
        }
        else
        {
			dist_ = -1.0;
			// C = 10.0;
			// residuals[0] = C*z;
            if (jacobians != NULL && jacobians[0] != NULL) 
            {
                jacobians[0][0] = 0;
                jacobians[0][1] = 0;
                // jacobians[0][2] = C;
                jacobians[0][2] = 0;
            }
// std::cout << ", Cost residual (out) ="<< residuals[0] << " , C= " << C << " , z= " << z;
        }

        return true;
  }

  private:
	Grid3d* g_3D;
	double sb;
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

class AutodiffTetherObstacleFunctor {

	public:
    AutodiffTetherObstacleFunctor(){}

		struct TetherObstacleFunctor 
		{
			TetherObstacleFunctor(double weight_factor_, Grid3d* grid_3D_, geometry_msgs::Vector3 pos_reel_ugv_, double sb_, bool write_data_, std::string user_name_)
			: wf(weight_factor_), g_3D(grid_3D_), pos_reel_ugv(pos_reel_ugv_), sb(sb_), w_d_(write_data_), user(user_name_), 
			_parableCostFunctor(new DistanceFunction(g_3D, sb)), _numPointFunctor(new NumPointFunction())
			{
			}

			template <typename T>
			bool operator()(const T* const stateUGV, const T* const stateUAV, const T* const params, T* residual) const 
			{
				T ugv_reel[4] = {stateUGV[0], stateUGV[1], stateUGV[2], stateUGV[3] + T{pos_reel_ugv.z}}; // Set first parable point on the reel position
				
				// Here is compute the Parable Parameters 
				T np_; 
				T u_x , u_y;
				T d_[1];
				T dist_[1];

				/* Map:
				params[1] = Xo 
				params[2] = Yo 
				params[3] = a 
				params[4] = length (Not used length because it is not optimized)
				*/
				T delta_x = (stateUAV[1] - ugv_reel[1]);
				T delta_y = (stateUAV[2] - ugv_reel[2]);
				T delta_z = (stateUAV[3] - ugv_reel[3]);
				dist_[0] = sqrt(delta_x * delta_x + delta_y * delta_y);
				d_[0] = sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z );
	
				if (sqrt(delta_x * delta_x + delta_y * delta_y) <0.001){
					u_x = u_y = T{0.0};
				} else{
					u_x = delta_x /dist_[0];
					u_y = delta_y /dist_[0];
				}

				if (d_[0] < 1.0) // To not make less than num_point_per_unit_length the value of points in parable
					d_[0] = T{1.0};

        		_numPointFunctor(d_, &np_); // To get the values and parameters needed for computing the parable interpolation

				// Here is compute the parable point and it cost 
				T point[3];
				T dist_obst_;	
				T parable_cost_;
				T cost_state_parable = T{0.0};
				T x_  =  T{0.0};
				T C;
				vector<T> v_dist;	
				v_dist.clear();
				int first_coll_, last_coll_;
				first_coll_ = last_coll_ = -1;

				for(int i = 0; i < np_; i++, x_ += dist_[0]/ np_ ){  
					point[0] = ugv_reel[1] + u_x * x_;
					point[1] = ugv_reel[2] + u_y * x_;
					point[2] = params[3] * cosh((x_ - params[1])/params[3]) +( params[2]-params[3]);

        			_parableCostFunctor(point, &dist_obst_);

					if(dist_obst_ < T{0.0}){
						// parable_cost_ = T{10.0} * T{point[2]} * T{point[2]} ;
						if (first_coll_ == -1)
							first_coll_ = i;
						last_coll_ = i;
					}else{
						if (dist_obst_ < T{sb}){
							if (first_coll_ == -1)
								first_coll_ = i;
							last_coll_ = i;
						}
					}
				v_dist.push_back(dist_obst_);
				}

				// Here we compute the cost of the catenary including if is between obstacles
				T distance_;
				for(int i = 0; i < np_; i++){ 
					if (v_dist[i]< T{0.001} && v_dist[i]>= T{0.0}) 
						distance_ = T{0.001};
					else
						distance_ = v_dist[i];

					T div = T{1.0}/distance_;
					if (i >=  first_coll_ && i <= last_coll_){
						if(v_dist[i] < T{0.0})
							C = T{-10000.0};
						else
							C = T{100.0};
					}else
						C = T{1.0};	
					parable_cost_ = div*C;
					cost_state_parable = cost_state_parable + parable_cost_; // To get point parable cost
				}

				if (np_> T{0.0})
					cost_state_parable = cost_state_parable/np_;
				else{
					cost_state_parable = T{10000.0};
					std::cout << "!!!!!!!!!!!!!!!!! COSA MAS RARA "<< std::endl << std::endl;
				}

				residual[0] = wf * cost_state_parable;
std::cout << "		["<< params[0]<<"] cost=" << cost_state_parable << " , coll=["<< first_coll_ << "/" << last_coll_ <<"]"<< std::endl ;
					
				return true;
			}

			bool w_d_;
			double wf, sb;
			geometry_msgs::Vector3 pos_reel_ugv;
			std::string user;
			Grid3d* g_3D;
	    	ceres::CostFunctionToFunctor<1, 3> _parableCostFunctor;
	    	ceres::CostFunctionToFunctor<1, 1> _numPointFunctor;
		};
	private:
};

#endif