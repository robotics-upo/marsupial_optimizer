#ifndef CERES_CONSTRAINS_REPULSION_UAV_HPP
#define CERES_CONSTRAINS_REPULSION_UAV_HPP

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

#include "misc/grid3d.hpp"
#include <octomap_msgs/Octomap.h> 
#include <octomap/OcTree.h>

using ceres::SizedCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
using ceres::HuberLoss;

class RepulsionFunctorUAV : public SizedCostFunction<1, 4>   {

public:
    RepulsionFunctorUAV(double weight_factor, double dist_fix, Eigen::Vector3d fix_pos_ref,
                        pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points)
                        : wf_(weight_factor), df(dist_fix), fpr(fix_pos_ref), kdT_(kdT_From_NN), o_p_(obstacles_Points)
    {}
    
    ~RepulsionFunctorUAV()
    {}

    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const 
    {
        double t = parameters[0][0];
        double x = parameters[0][1];
        double y = parameters[0][2];
        double z = parameters[0][3];
        double d , dp;
            
        d = sqrt( (fpr.x() - x)*(fpr.x() - x) + (fpr.y() - y)*(fpr.y() - y) + (fpr.z() - z)*(fpr.z() - z) );   
        
        NearNeighbor nn;
        double dp_x, dp_y, dp_z;
        nn.nearestObstacleStateCeres(kdT_ , x, y, z, o_p_, dp_x, dp_y, dp_z);
        dp = sqrt( (dp_x - x)*(dp_x - x) + (dp_y - y)*(dp_y - y) + (dp_z - z)*(dp_z - z) );  

        // printf("node[%f] , df=[%f]\n",t,df);
        residuals[0] = wf_ * exp(pow((df + d) - (dp) ,2));
        printf("RepulsionFunctorUAV[%f]: residuals[0] =[%f]  (df + d) = (%f + %f) dp=%f\n",t,residuals[0], df, d, dp);

        if (jacobians != NULL && jacobians[0] != NULL) 
        {
            if (d < 0.0001 || dp < 0.0001){ 
                jacobians[0][0] = 0.0;
                jacobians[0][1] = 0.0;
                jacobians[0][2] = 0.0;
                jacobians[0][3] = 0.0;
            }else{
                jacobians[0][0] = 0.0;
                jacobians[0][1] = 2.0*wf_*exp((df - dp + d)*(df - dp + d))*((2.0*dp_x - 2.0*x)/(2.0*dp) - (2.0*fpr.x() - 2.0*x)/(2.0*d))*(df - dp + d);
                jacobians[0][2] = 2.0*wf_*exp((df - dp + d)*(df - dp + d))*((2.0*dp_y - 2.0*y)/(2.0*dp) - (2.0*fpr.y() - 2.0*y)/(2.0*d))*(df - dp + d);
                jacobians[0][3] = 2.0*wf_*exp((df - dp + d)*(df - dp + d))*((2.0*dp_z - 2.0*z)/(2.0*dp) - (2.0*fpr.z() - 2.0*z)/(2.0*d))*(df - dp + d);
            }
        // printf("RepulsionFunctorUAV: jacobians[0]=[%f,%f,%f,%f]\n",jacobians[0][0],jacobians[0][1],jacobians[0][2],jacobians[0][3]);
        }
        return true;
    }
    double wf_, df;
    Eigen::Vector3d fpr;
    pcl::KdTreeFLANN <pcl::PointXYZ> kdT_;
    pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_;
    float s_, s_i_;

    Grid3d* g_3D_;

private:

};

#endif