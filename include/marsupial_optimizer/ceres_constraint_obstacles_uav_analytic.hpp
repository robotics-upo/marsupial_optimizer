#ifndef CERES_CONSTRAINS_OBSTACLET_UAV_ANALYTIC_HPP
#define CERES_CONSTRAINS_OBSTACLET_UAV_ANALYTIC_HPP

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include "catenary_checker/grid3d.hpp"

using ceres::SizedCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
using ceres::HuberLoss;


class ObstacleDistanceFunctorUAVAnalytic : public SizedCostFunction<1, 4>  
{
public:
    ObstacleDistanceFunctorUAVAnalytic(double weight_factor, double safty_bound, Grid3d* grid_3D_)
                     : wf_(weight_factor), sb_(safty_bound), g_3D_(grid_3D_) 
    {
    }

    virtual ~ObstacleDistanceFunctorUAVAnalytic(void) 
    {
    }

    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const 
    {
        double p = parameters[0][0];
        double x = parameters[0][1];
        double y = parameters[0][2];
        double z = parameters[0][3];
        double f_slope_ = 5.0;

        TrilinearParams d = g_3D_->getPointDistInterpolation(x, y, z);

        // // Obtenemos parametros de interpolación del grid en X, y, Z
        // double d_ = (d.a0 + d.a1*x + d.a2*y + d.a3*z + d.a4*x*y + d.a5*x*z + d.a6*y*z + d.a7*x*y*z); 
        // double d_sb_ = sb_ * 1.5;
        // if (d_ > sb_)
        //     f_slope_ = 0.0;
        // else
        //     f_slope_ = 5.0;
        // if (d_ > sb_){
        //     residuals[0] =0.0;
        //     if (jacobians != NULL && jacobians[0] != NULL) 
        //     {
        //         jacobians[0][0] = 0.0;
        //         jacobians[0][1] = 0.0;
        //         jacobians[0][2] = 0.0;
        //         jacobians[0][3] = 0.0;
        //     }
        // }
        // else{
        //     residuals[0] = wf_ * exp(f_slope_*( d_sb_ - d_));
        //     if (jacobians != NULL && jacobians[0] != NULL) 
        //     {
        //         jacobians[0][0] = 0.0;
        //         jacobians[0][1] = -f_slope_*wf_*exp(-f_slope_*(d.a0 - d_sb_ + d.a1*x + d.a2*y + d.a3*z + d.a4*x*y + d.a5*x*z + d.a6*y*z + d.a7*x*y*z))*(d.a1 + d.a4*y + d.a5*z + d.a7*y*z);
        //         jacobians[0][2] = -f_slope_*wf_*exp(-f_slope_*(d.a0 - d_sb_ + d.a1*x + d.a2*y + d.a3*z + d.a4*x*y + d.a5*x*z + d.a6*y*z + d.a7*x*y*z))*(d.a2 + d.a4*x + d.a6*z + d.a7*x*z);
        //         jacobians[0][3] = -f_slope_*wf_*exp(-f_slope_*(d.a0 - d_sb_ + d.a1*x + d.a2*y + d.a3*z + d.a4*x*y + d.a5*x*z + d.a6*y*z + d.a7*x*y*z))*(d.a3 + d.a5*x + d.a6*y + d.a7*x*y);
        //     }
        // }


        // Obtenemos parametros de interpolación del grid en X, y, Z
        double d_ = (d.a0 + d.a1*x + d.a2*y + d.a3*z + d.a4*x*y + d.a5*x*z + d.a6*y*z + d.a7*x*y*z); 
        double d_sb_ = sb_ * 1.05;
        
        double max_value_residual = 100.0;
        double min_value_residual = 0.0;
        double m ;
        if (d_ > d_sb_)
            m = 0.0;
        else
            m = (max_value_residual- min_value_residual)/( 0 - d_sb_);

        residuals[0] = wf_ * m *(d_ - d_sb_);
        
        if (jacobians != NULL && jacobians[0] != NULL) 
        {
            jacobians[0][0] = 0.0;
            jacobians[0][1] = m*wf_*(d.a1 + d.a4*y + d.a5*z + d.a7*y*z);
            jacobians[0][2] = m*wf_*(d.a2 + d.a4*x + d.a6*z + d.a7*x*z);
            jacobians[0][3] = m*wf_*(d.a3 + d.a5*x + d.a6*y + d.a7*x*y);
        }
        printf("ObstacleDistanceFunctorUAVAnalytic[%f]: residuals[0] =%f , d_=[%f/%f]\n",p, residuals[0], d_, d_sb_);
        return true;
    }
    double wf_, sb_;
    Grid3d* g_3D_;

private:
};

#endif