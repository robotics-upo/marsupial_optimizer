#ifndef CERES_CONSTRAINS_OBSTACLES_UGV_ANALYTIC_HPP
#define CERES_CONSTRAINS_OBSTACLES_UGV_ANALYTIC_HPP

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include "catenary_checker/near_neighbor.hpp"

using ceres::SizedCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
using ceres::HuberLoss;

class ObstacleDistanceFunctorUGVAnalytic : public SizedCostFunction<1, 4> 
{

public:
    ObstacleDistanceFunctorUGVAnalytic(double weight_factor, double safty_bound, double count_fix_points_ugv, 
                               pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points)
                               : wf_(weight_factor), sb_(safty_bound), cfp_ugv(count_fix_points_ugv), kdT(kdT_From_NN), o_p(obstacles_Points)
    {}
    
    ~ObstacleDistanceFunctorUGVAnalytic()
    {}

    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const 
    {
        // To avoid obstacles
        double p = parameters[0][0];
        double x = parameters[0][1];
        double y = parameters[0][2];
        double z = parameters[0][3];
        double factor_ = 1000.0;
        double factor_exp = 4.0;

        double d_ugv_1;
        double n_x, n_y, n_z;

        NearNeighbor nn;
        nn.nearestObstacleStateCeres(kdT , x, y, z, o_p, n_x, n_y, n_z);
    
        d_ugv_1 = sqrt( (n_x - x)*(n_x - x) + (n_y - y)*(n_y - y) + (n_z - z)*(n_z - z) );
            
        residuals[0] = wf_ * factor_ * log(1.0 + exp(factor_exp*(sb_ - d_ugv_1)) ) ;

        if (p >= cfp_ugv){
            if (jacobians != NULL && jacobians[0] != NULL) 
            {
                jacobians[0][0] = 0.0;
                jacobians[0][1] = (factor_*factor_exp*wf_*exp(factor_exp*(sb_ - sqrt(d_ugv_1)))*(2.0*n_x - 2.0*x))/(2.0*(exp(factor_exp*(sb_ - sqrt(d_ugv_1))) + 1)*sqrt(d_ugv_1));
                jacobians[0][2] = (factor_*factor_exp*wf_*exp(factor_exp*(sb_ - sqrt(d_ugv_1)))*(2.0*n_y - 2.0*y))/(2.0*(exp(factor_exp*(sb_ - sqrt(d_ugv_1))) + 1)*sqrt(d_ugv_1));
                jacobians[0][3] = (factor_*factor_exp*wf_*exp(factor_exp*(sb_ - sqrt(d_ugv_1)))*(2.0*n_z - 2.0*z))/(2.0*(exp(factor_exp*(sb_ - sqrt(d_ugv_1))) + 1)*sqrt(d_ugv_1));
            }
        }
        
        return true;
    }
    double wf_, sb_, cfp_ugv;
    pcl::KdTreeFLANN <pcl::PointXYZ> kdT;
    pcl::PointCloud <pcl::PointXYZ>::Ptr o_p;

private:

};

#endif