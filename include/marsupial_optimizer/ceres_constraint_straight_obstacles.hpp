#ifndef CERES_CONSTRAINS_STRAIGHT_OBSTACLES_AUTODIFF_HPP
#define CERES_CONSTRAINS_STRAIGHT_OBSTACLES_AUTODIFF_HPP

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

class DistanceStraightFunction : public ceres::SizedCostFunction<1, 3> 
{
 public:

    DistanceStraightFunction(Grid3d* grid_, double security_dist_)
      : g_3D(grid_), sb(security_dist_)
    {
    }

    virtual ~DistanceStraightFunction(void) 
    {
    }
	
    virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const 
    {
        double x = parameters[0][0];
        double y = parameters[0][1];
        double z = parameters[0][2];
		double dist_, div;
		double C;

		if(g_3D->isIntoMap(x, y, z))
        {
			TrilinearParams p = g_3D->getPointDistInterpolation(x, y, z);
            dist_ = p.a0 + p.a1*x + p.a2*y + p.a3*z + p.a4*x*y + p.a5*x*z + p.a6*y*z + p.a7*x*y*z;
			if(dist_ < 0.0001)
				dist_ = 0.0001;
			div = 1/dist_;
			if (dist_ < sb)
				C = 10.0;
			else
				C = 4.0;	
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
			C = 100000.0;
			residuals[0] = C*z;
            if (jacobians != NULL && jacobians[0] != NULL) 
            {
                jacobians[0][0] = 0;
                jacobians[0][1] = 0;
                jacobians[0][2] = C;
            }
        }

        return true;
  }

  private:
	Grid3d* g_3D;
	double sb;
};

class NumPointStraightFunction : public ceres::SizedCostFunction<1, 1> 
{
 public:

    NumPointStraightFunction(void)
    {
    }

    virtual ~NumPointStraightFunction(void) 
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

class AutodiffStraightFunctor {

	public:
    AutodiffStraightFunctor(){}

		struct StraightFunctor 
		{
			StraightFunctor(double weight_factor_, Grid3d* grid_3D_, geometry_msgs::Vector3 pos_reel_ugv_, double sb_)
			: wf(weight_factor_), g_3D(grid_3D_), pos_reel_ugv(pos_reel_ugv_), sb(sb_), 
			_straigthCostFunctor(new DistanceStraightFunction(g_3D, sb)), _numPointFunctor(new NumPointStraightFunction())
			{
			}

			template <typename T>
			bool operator()(const T* const pUGV, const T* const pUAV, const T* const length, T* residual) const 
			{
				T ugv_reel[4] = {pUGV[0], pUGV[1], pUGV[2], pUGV[3] + T{pos_reel_ugv.z}}; // Set first Straigth point on the reel position
				
				// Here is compute the Straigth Parameters 
				T d_[1];
				T np_; 

				// About distance between UGV and UAV in the plane
				d_[0] = sqrt(pow(pUAV[1]-ugv_reel[1],2)+pow(pUAV[2]-ugv_reel[2],2)+pow(pUAV[3]-ugv_reel[3],2)); 
				
				if (d_[0] < 1.0) // To not make less than num_point_per_unit_length the value of points in Straigth
					d_[0] = T{1.0};
				
				T dx_ = pUAV[1]-ugv_reel[1];
				T dy_ = pUAV[2]-ugv_reel[2];
				T dz_ = pUAV[3]-ugv_reel[3];

        		_numPointFunctor(d_, &np_); // To get the values and parameters needed for computing the Straigth interpolation

				// Here is compute the Straigth point and it cost 
				T point[3];
				T straigth_cost_;	
				T cost_state_straigth = T{0.0};
				for(int i = 0; i < np_; i++){  
					T step_x_ = dx_ / np_;
					T step_y_ = dy_ / np_;
					T step_z_ = dz_ / np_;
					point[0] = ugv_reel[1]+ step_x_* (double)i; 
					point[1] = ugv_reel[2]+ step_y_* (double)i; 
					point[2] = ugv_reel[3]+ step_z_* (double)i;    	
        			_straigthCostFunctor(point, &straigth_cost_);

					cost_state_straigth = cost_state_straigth + straigth_cost_; // To get point straigth cost
				}
				residual[0] = wf * 1.0 * cost_state_straigth;
				
				return true;
			}

			double wf, sb;
			geometry_msgs::Vector3 pos_reel_ugv;
			Grid3d* g_3D;
	    	ceres::CostFunctionToFunctor<1, 3> _straigthCostFunctor;
	    	ceres::CostFunctionToFunctor<1, 1> _numPointFunctor;
		};
	private:
};

#endif