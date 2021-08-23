#ifndef CERES_CONSTRAINS_TRAVERSABILITY_ANALYTIC_UGV_HPP
#define CERES_CONSTRAINS_TRAVERSABILITY_ANALYTIC_UGV_HPP

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include "misc/near_neighbor.hpp"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;


class TraversabilityDistanceFunctorUGVAnalytic : public SizedCostFunction<1, 4>  
{

public:
    TraversabilityDistanceFunctorUGVAnalytic(double weight_factor, double count_fix_points_ugv,
                                     pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points)
                                    : wf_(weight_factor), kdT(kdT_From_NN), o_p(obstacles_Points), cfp_ugv(count_fix_points_ugv)
    {}
    
    ~TraversabilityDistanceFunctorUGVAnalytic()
    {}

    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const 
    {
        double p = parameters[0][0];
        double x = parameters[0][1];
        double y = parameters[0][2];
        double z = parameters[0][3];
        double factor_ = 1000.0;
        
        double d_ugv_;
        double n_x, n_y, n_z;

        NearNeighbor nn;
        nn.nearestObstacleStateCeres(kdT , x, y, z, o_p, n_x, n_y, n_z);

        d_ugv_ = (z - n_z)*(z - n_z); // To the closest point to traversavility PC is only consider distance in Z axe

        double bound_dist = 0.01;
        double bound = exp(bound_dist); // To make 0.0 residual 
            
        residuals[0] =  wf_ * factor_ * (exp(4.0*(d_ugv_)) - bound);

        if (p >= cfp_ugv){
            if (jacobians != NULL && jacobians[0] != NULL) 
            {
                jacobians[0][0] = 0.0;
                jacobians[0][1] = 0.0;
                jacobians[0][2] = 0.0;
                jacobians[0][3] = - wf_ * factor_ * exp(4.0*(d_ugv_))*(8*n_z - 8*z);
            }
        }
        return true;
    }

    double wf_, cfp_ugv;
    pcl::KdTreeFLANN <pcl::PointXYZ> kdT;
    pcl::PointCloud <pcl::PointXYZ>::Ptr o_p;

private:

};

#endif