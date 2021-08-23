#ifndef CERES_CONSTRAINT_OBSTACLES_UAV_HPP
#define CERES_CONSTRAINT_OBSTACLES_UAV_HPP

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

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;


class ObstacleDistanceFunctorUAV {

public:
    ObstacleDistanceFunctorUAV(){}
    
    struct ComputeDistanceObstaclesUAV 
    {
        ComputeDistanceObstaclesUAV (Grid3d* _grid_3D) 
                                    :  _g_3D(_grid_3D)
        {
        }
        bool operator()(const double *state, double *dist_) const 
        {
            double x = state[1];
            double y = state[2];
            double z = state[3];
            // std::cout << "statePos=["<< x <<" , " << y << " , " << z << "] "<< std::endl;

            TrilinearParams d = _g_3D->getPointDistInterpolation(x ,y ,z);
            double d_ = (d.a0 + d.a1*x + d.a2*y + d.a3*z + d.a4*x*y + d.a5*x*z + d.a6*y*z + d.a7*x*y*z); 
            // d_[0] = d;

            dist_[0] = d_;
            dist_[1] = 0.0;
            dist_[2] = 0.0;
            dist_[3] = 0.0;

            // std::cout <<" , d_[0]="<< dist_[0] << std::endl;
            return true;
        }

        Grid3d* _g_3D;
    };

    struct ObstaclesFunctor 
    {
        ObstaclesFunctor(double weight_factor, double safty_bound,  Grid3d* grid_3D_)
                         : wf_(weight_factor), sb_(safty_bound), g_3D_(grid_3D_)
        {
            compute_nearest_distance.reset(new ceres::CostFunctionToFunctor<4,4>(
                                    new ceres::NumericDiffCostFunction<ComputeDistanceObstaclesUAV,
                                                                        ceres::CENTRAL, 
                                                                        4,
                                                                        4>( 
                                    new ComputeDistanceObstaclesUAV(g_3D_))));
        }

        template <typename T>
        bool operator()(const T* const statePos1, T* residual) const 
        {
            T d_uav, n_[4], Diff_;
            T state[4]={statePos1[0], statePos1[1], statePos1[2], statePos1[3]};
            double f_slope_ = 5.0;
            
            // std::cout << "statePos1["<<state[0]<<"] = [" <<state[1]<< "," <<state[2]<< "," <<state[3] <<"] "<< std::endl;
            (*compute_nearest_distance)(state, n_);
            d_uav = n_[0];

            Diff_ = (sb_*1.5 - d_uav);
            // residual[0] = wf_ * 100.0 * log(1.0 + exp(4.0*Diff_) ) ;
            if(sb_ < d_uav)
                residual[0] = T{0.0};
            else    
                residual[0] = wf_ * exp(f_slope_*(Diff_));

            std::cout << " , ["<<statePos1[0] <<"] residual[0]= " << residual[0] << " , d_uav= " << d_uav << std::endl;

            return true;
        }

        double wf_, sb_;
        Grid3d* g_3D_;
        std::unique_ptr<ceres::CostFunctionToFunctor<4,4> > compute_nearest_distance;
    };


private:

};

#endif