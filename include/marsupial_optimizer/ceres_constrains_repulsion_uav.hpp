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
    RepulsionFunctorUAV(double weight_factor, double safty_bound, float step_, float step_inv_, Grid3d* grid_3D_, Eigen::Vector3d fix_pos_ref)
            : wf_(weight_factor), sb_(safty_bound), s_(step_), s_i_(step_inv_), g_3D_(grid_3D_), fpr(fix_pos_ref)
    {}
    
    ~RepulsionFunctorUAV()
    {}

    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const 
    {
        double t = parameters[0][0];
        double x = parameters[0][1];
        double y = parameters[0][2];
        double z = parameters[0][3];
        double df, d , d_arg;
            
        
        df = g_3D_->getPointDist(fpr.x(),fpr.y(),fpr.z());
        d_arg = ( (fpr.x() - x)*(fpr.x() - x) + (fpr.y() - y)*(fpr.y() - y) + (fpr.z() - z)*(fpr.z() - z) );   
        if (d_arg < 0.0001)
            d = 0.0;
        else
            d = sqrt(d_arg);
        TrilinearParams dp = g_3D_->getPointDistInterpolation(x, y, z);
        double value_inter = (dp.a0 + dp.a1*x + dp.a2*y + dp.a3*z + dp.a4*x*y + dp.a5*x*z + dp.a6*y*z + dp.a7*x*y*z);

        residuals[0] = wf_ * pow((df + d) - (value_inter) ,2);

        if (jacobians != NULL && jacobians[0] != NULL) 
        {
            if (d > 0.0001){ 
                jacobians[0][0] = 0.0;
                jacobians[0][1] = 2.0*wf_*(dp.a1 + (2.0*fpr.x() - 2.0*x)/(2.0*sqrt((fpr.x() - x)*(fpr.x() - x) + (fpr.y() - y)*(fpr.y() - y) + (fpr.z() - z)*(fpr.z() - z))) + dp.a4*y + dp.a5*z + dp.a7*y*z) * (dp.a0 - df + dp.a1*x + dp.a2*y + dp.a3*z - sqrt((fpr.x() - x)*(fpr.x() - x) + (fpr.y() - y)*(fpr.y() - y) + (fpr.z() - z)*(fpr.z() - z)) + dp.a4*x*y + dp.a5*x*z + dp.a6*y*z + dp.a7*x*y*z);
                jacobians[0][2] = 2.0*wf_*(dp.a2 + (2.0*fpr.y() - 2.0*y)/(2.0*sqrt((fpr.x() - x)*(fpr.x() - x) + (fpr.y() - y)*(fpr.y() - y) + (fpr.z() - z)*(fpr.z() - z))) + dp.a4*x + dp.a6*z + dp.a7*x*z) * (dp.a0 - df + dp.a1*x + dp.a2*y + dp.a3*z - sqrt((fpr.x() - x)*(fpr.x() - x) + (fpr.y() - y)*(fpr.y() - y) + (fpr.z() - z)*(fpr.z() - z)) + dp.a4*x*y + dp.a5*x*z + dp.a6*y*z + dp.a7*x*y*z);
                jacobians[0][3] = 2.0*wf_*(dp.a3 + (2.0*fpr.z() - 2.0*z)/(2.0*sqrt((fpr.x() - x)*(fpr.x() - x) + (fpr.y() - y)*(fpr.y() - y) + (fpr.z() - z)*(fpr.z() - z))) + dp.a5*x + dp.a6*y + dp.a7*x*y) * (dp.a0 - df + dp.a1*x + dp.a2*y + dp.a3*z - sqrt((fpr.x() - x)*(fpr.x() - x) + (fpr.y() - y)*(fpr.y() - y) + (fpr.z() - z)*(fpr.z() - z)) + dp.a4*x*y + dp.a5*x*z + dp.a6*y*z + dp.a7*x*y*z);
            }
            else{
                jacobians[0][0] = 0.0;
                jacobians[0][1] = 0.0;
                jacobians[0][2] = 0.0;
                jacobians[0][3] = 0.0;
            }
        }

        return true;
    }
    double wf_, sb_;
    Eigen::Vector3d fpr;
    float s_, s_i_;

    Grid3d* g_3D_;

private:

};

#endif