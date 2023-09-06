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
​    DistanceFunction(Grid3d* grid_, double security_dist_)
      : g_3D(grid_), sb(security_dist_)
    {
    }
​
    virtual ~DistanceFunction(void) 
    {
    }
​
    virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const 
    {
        double x = parameters[0][0];
        double y = parameters[0][1];
        double z = parameters[0][2];
		double dist_, div;
		double C;
​
		if(g_3D->isIntoMap(x, y, z))
        {
			TrilinearParams p = g_3D->getPointDistInterpolation(x, y, z);
            dist_ = p.a0 + p.a1*x + p.a2*y + p.a3*z + p.a4*x*y + p.a5*x*z + p.a6*y*z + p.a7*x*y*z;
			if(dist_ < 0.0001)
				dist_ = 0.0001;
			div = 1/dist_;
			if (dist_ < sb)
				C = 2.0;
			else
				C = 1.0;	
			residuals[0] = div*C;
            if (jacobians != NULL && jacobians[0] != NULL) 
            {
                jacobians[0][0] = -C*(p.a1 + p.a4*y + p.a5*z + p.a7*y*z)*div*div;
                jacobians[0][1] = -C*(p.a2 + p.a4*x + p.a6*z + p.a7*x*z)*div*div;
                jacobians[0][2] = -C*(p.a3 + p.a5*x + p.a6*y + p.a7*x*y)*div*div;
            }
        }
        else
        {
            residuals[0] = 1000000.0;
            if (jacobians != NULL && jacobians[0] != NULL) 
            {
                jacobians[0][0] = 0;
                jacobians[0][1] = 0;
                jacobians[0][2] = 0;
            }
        }
​
        return true;
  }
​
  private:
	Grid3d* g_3D;
	double sb;
};

class AutodiffParableFunctor {

	public:
    AutodiffParableFunctor(){}

		struct NumPointParable 
		{
			NumPointParable()
			{}
			bool operator()(const double *dist, double *np) const 
			{
				int num_point_per_unit_length = 10;
				int num_point_parable = round( (double)num_point_per_unit_length * dist[0]);

				np[0] = num_point_parable;

				return true;
			}
		};

		struct ParableFunctor 
		{
			ParableFunctor(double weight_factor_, Grid3d* grid_3D_, geometry_msgs::Vector3 pos_reel_ugv_, double sb_, bool write_data_, std::string user_name_)
			: wf(weight_factor_), g_3D(grid_3D_), pos_reel_ugv(pos_reel_ugv_), sb(sb_), w_d_(write_data_), user(user_name_), _parableCostFunctor(new DistanceFunction(g_3D, sb))
			{
				point_parable.reset(new ceres::CostFunctionToFunctor<1,1>(
							  new ceres::NumericDiffCostFunction<NumPointParable, ceres::CENTRAL,1,1>(new NumPointParable())));
			}

			template <typename T>
			bool operator()(const T* const pUGV, const T* const pUAV, const T* const param, T* residual) const 
			{
				T ugv_reel[4] = {pUGV[0], pUGV[1], pUGV[2], pUGV[3] + T{pos_reel_ugv.z}}; // Set first parable point on the reel position
				
				// Here is compute the Parable Parameters 
				T d_[1];
				T np[1]; 
				T u_x , u_y;
				T fix_value = T{0.01};
				bool change_x, change_y;
				change_x = change_y = true;

				if(pUAV[1] > pUGV[1])
					u_x = T{1.0};
				else if(pUAV[1] < pUGV[1])
					u_x = T{-1.0};
				else
					u_x = T{0.0};    
				if(pUAV[2] > pUGV[2])
					u_y = T{1.0};
				else if(pUAV[2] < pUGV[2])
					u_y = T{-1.0};
				else
					u_y = T{0.0};  
				if ((pUGV[1] - pUAV[1]) < fix_value && (pUGV[1] - pUAV[1]) > T{-1.0}*fix_value) 
					change_x = false;
				if ((pUGV[2] - pUAV[2]) < fix_value && (pUGV[2] - pUAV[2]) > T{-1.0}*fix_value)
					change_y = false;

				// About distance between UGV and UAV in the plane
				if ( !change_x && !change_y )  //Not change in X-Y plane, so parable can't be compute
					d_[0] = sqrt(pow(pUAV[3]-ugv_reel[3],2)); 
				else
					d_[0] = sqrt(pow(pUAV[1]-ugv_reel[1],2)+pow(pUAV[2]-ugv_reel[2],2)); 
				
				if (d_[0] < 1.0) // To not make less than num_point_per_unit_length the value of points in parable
					d_[0] = T{1.0};

				(*point_parable)(d_, np); // To get the values and parameters needed for computing the parable interpolation

				// Here is compute the parable point and it cost 
				T point[3];
				T parable_cost_;	
				T cost_state_parable = T{0.0};
				T x_  =  T{0.0};
				T tetha_ = atan((pUAV[2] - pUGV[2])/(pUAV[1] - pUGV[1]));
				for(int i = 0; i < np[0]; i++){  
					if ( !change_x && !change_y ){ // To check difference position between UGV and UAV only in z-axe, so parable it is not computed
						T _step = d_[0] / T{np[0]};
						point[0] = pUGV[1];
						point[1] = pUGV[2];
						point[2] = pUGV[3]+ _step* (double)i;    
					}
					else{ 	// In case that UGV and UAV are not in the same point the plane X-Y
						x_ = x_ + (d_[0]/ T{np[0]});
						point[0] = pUGV[1] + u_x * cos(tetha_) * x_;
						point[1] = pUGV[2] + u_y * sin(tetha_) * x_;
						point[2] = param[1] * x_* x_ + param[2] * x_ + param[3];
					}
        			_parableCostFunctor(point, &parable_cost_);

					cost_state_parable = cost_state_parable + parable_cost_; // To get point parable cost
					// std::cout << "[" << np[0] <<"/"<< i<<"] , parable_cost_[0]=" << parable_cost_[0]  << " PARABLE" << std::endl;
				}
				// std::cout << "[" << pUGV[0] <<"] , cost_state_parable=" << cost_state_parable << std::endl;
				residual[0] = wf * 0.001 * cost_state_parable;
					
				return true;
			}

			std::unique_ptr<ceres::CostFunctionToFunctor<1,1> > point_parable;
			bool w_d_, sb;
			double wf;
			geometry_msgs::Vector3 pos_reel_ugv;
			std::string user;
			Grid3d* g_3D;
	    	ceres::CostFunctionToFunctor<1, 3> _parableCostFunctor;
		};
	private:
};

#endif