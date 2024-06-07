#ifndef CERES_CONSTRAINS_OBSTACLES_UAV_HPP
#define CERES_CONSTRAINS_OBSTACLES_UAV_HPP

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include "catenary_checker/near_neighbor.hpp"
#include "catenary_checker/grid3d.hpp"

using ceres::AutoDiffCostFunction;
using ceres::NumericDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

#include <iostream>
#include <fstream>
#include <string>


class ObstacleDistanceFunctorUAV : public ceres::SizedCostFunction<1, 3> 
{
public:
    ObstacleDistanceFunctorUAV(Grid3d* grid_)
      : g_3D(grid_){}
    
    virtual ~ObstacleDistanceFunctorUAV(void) 
    {
    }
    
    virtual bool Evaluate(double const* const* parameters,
                    double* residuals,
                    double** jacobians) const 
    {
    
        double x_ = parameters[0][0];
        double y_ = parameters[0][1];
        double z_ = parameters[0][2];
        double d_;

		if (g_3D->isIntoMap(x_,y_,z_)){
			TrilinearParams p = g_3D->getPointDistInterpolation(x_,y_,z_);
            d_ = p.a0 + p.a1*x_ + p.a2*y_ + p.a3*z_ + p.a4*x_*y_ + p.a5*x_*z_ + p.a6*y_*z_ + p.a7*x_*y_*z_;
		
			residuals[0] = d_;

            if (jacobians != NULL && jacobians[0] != NULL) 
            {
				jacobians[0][0] = p.a1 + p.a4*y_ + p.a5*z_ + p.a7*y_*z_;
                jacobians[0][1] = p.a2 + p.a4*x_ + p.a6*z_ + p.a7*x_*z_;
                jacobians[0][2] = p.a3 + p.a5*x_ + p.a6*y_ + p.a7*x_*y_;
            }
        }
		else{
			d_ = -1.0;
	
            residuals[0] = d_; 
        }  
            return true;
    }
    private:
		Grid3d* g_3D;
};

class AutodiffObstaclesFunctor 
{
    public:
    AutodiffObstaclesFunctor(){}
    
    struct ObstaclesFunctor
    {
        ObstaclesFunctor(double weight_factor, double safty_bound, Grid3d* grid_3D_)
        : wf_(weight_factor), sb_(safty_bound), g_3D(grid_3D_), compute_nearest_distance(new ObstacleDistanceFunctorUAV(g_3D))
        {
        }

        template <typename T>
        bool operator()(const T* const statePos1, T* residual) const 
        {
            // To avoid obstacles
            T d_, arg_d, Diff_;
            T point[3];
            point[0] = statePos1[1];
            point[1] = statePos1[2];
            point[2] = statePos1[3];
            double f_slope_ = 5.0;
            
            compute_nearest_distance(point, &d_);

            T d_sb_ = sb_ * T{1.05};
            T max_value_residual = T{100.0};
            T min_value_residual = T{0.0};
            T max_value_dependent = T{0.0};
            T m ;
            if (d_ > d_sb_)
                m = T{0.0};
            else
                m = (max_value_residual- min_value_residual)/(max_value_dependent - d_sb_);
                
            residual[0] = wf_ * m *(d_ - d_sb_);

            return true;
        }

        double wf_, sb_;
        ceres::CostFunctionToFunctor<1,3> compute_nearest_distance;
		Grid3d* g_3D;
    };

    private:

};

#endif