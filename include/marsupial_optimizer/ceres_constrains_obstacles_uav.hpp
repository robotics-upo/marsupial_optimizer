#ifndef CERES_CONSTRAINS_OBSTACLES__UAV_HPP
#define CERES_CONSTRAINS_OBSTACLES__UAV_HPP

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include "misc/near_neighbor.hpp"
#include "misc/grid3d.hpp"
#include <octomap_msgs/Octomap.h> 
#include <octomap/OcTree.h>

using ceres::SizedCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
using ceres::HuberLoss;


class ObstacleDistanceFunctorUAV : public SizedCostFunction<1, 4>  
{
public:
    ObstacleDistanceFunctorUAV(double weight_factor, double safty_bound, float step_, float step_inv_, Grid3d* grid_3D_, pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, 
                     pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points)
                     : wf_(weight_factor), sb_(safty_bound), s_(step_), s_i_(step_inv_), g_3D_(grid_3D_) , kdT_(kdT_From_NN), o_p_(obstacles_Points)
    {
    }

    virtual ~ObstacleDistanceFunctorUAV(void) 
    {
    }

    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const 
    {
        double t = parameters[0][0];
        double x = parameters[0][1];
        double y = parameters[0][2];
        double z = parameters[0][3];
        double factor_ = 100.0;
        double f_slope_ = 4.0;


        bool is_into_ = g_3D_->isIntoMap(x, y, z);
        double d_;
        if (is_into_ )
		    d_ = g_3D_->getPointDist(x, y, z);
        else
            d_ = -1.0;
		
        TrilinearParams d = g_3D_->getPointDistInterpolation(x, y, z);

        // Obtenemos parametros de interpolación del grid en X, y, Z
        double value_ = (d.a0 + d.a1*x + d.a2*y + d.a3*z + d.a4*x*y + d.a5*x*z + d.a6*y*z + d.a7*x*y*z); 
        double value_sb_ = sb_*sb_;
        double value_exp_ =exp(-f_slope_*(- value_sb_ + value_));
        residuals[0] = wf_ * factor_ * log(1.0 + value_exp_ ) ;

        if (jacobians != NULL && jacobians[0] != NULL) 
        {
            jacobians[0][0] = 0.0;
            jacobians[0][1] = -(f_slope_*factor_*wf_*value_exp_*(d.a1 + d.a4*y + d.a5*z + d.a7*y*z))/(value_exp_ + 1.0);
            jacobians[0][2] = -(f_slope_*factor_*wf_*value_exp_*(d.a2 + d.a4*x + d.a6*z + d.a7*x*z))/(value_exp_ + 1.0);
            jacobians[0][3] = -(f_slope_*factor_*wf_*value_exp_*(d.a3 + d.a5*x + d.a6*y + d.a7*x*y))/(value_exp_ + 1.0);
        }

        return true;
    }

    double wf_, sb_;
    float s_, s_i_;
    Grid3d* g_3D_;
    pcl::KdTreeFLANN <pcl::PointXYZ> kdT_;
    pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_;

private:

};

#endif