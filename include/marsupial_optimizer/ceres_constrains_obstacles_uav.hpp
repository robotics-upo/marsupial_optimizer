#ifndef CERES_CONSTRAINS_OBSTACLES_UAV_HPP
#define CERES_CONSTRAINS_OBSTACLES_UAV_HPP

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
        ComputeDistanceObstaclesUAV (Grid3d *_grid_3D): _g_3D(_grid_3D)
        {
        }
        bool operator()(const double *state1, double *near_) const 
        {

            bool is_into_ = _g_3D->isIntoMap(state1[1], state1[2], state1[3]);
			double d_[1];
            if (is_into_)
				d_[0] = _g_3D->getPointDist((double)state1[1], (double)state1[2], (double)state1[3]);
			else
				d_[0] = -1.0;

            near_[0] = d_[0];

            return true;
        }

        double f_, sb_;
        Grid3d *_g_3D;
    };

    struct ObstaclesFunctor 
    {
        ObstaclesFunctor(double weight_factor, double safty_bound, Grid3d *grid_3D_): wf_(weight_factor), sb_(safty_bound), g_3D_(grid_3D_)
        {
            compute_nearest_distance.reset(new ceres::CostFunctionToFunctor<4,1>(
                                    new ceres::NumericDiffCostFunction<ComputeDistanceObstaclesUAV,
                                                                        ceres::CENTRAL, 
                                                                        4,
                                                                        1>( 
                                    new ComputeDistanceObstaclesUAV(g_3D_))));
        }

        template <typename T>
        bool operator()(const T* const statePos1, T* residual) const 
        {
            // To avoid obstacles
            T d_uav_1;
            T arg_d_uav_1;
            T n1_[1];

            (*compute_nearest_distance)(statePos1, n1_);
            arg_d_uav_1 = (statePos1[1]-n1_[0])*(statePos1[1]-n1_[0]) + (statePos1[2]-n1_[1])*(statePos1[2]-n1_[1]) + (statePos1[3]-n1_[2])*(statePos1[3]-n1_[2]);
            
            if (arg_d_uav_1 < 0.0001 && arg_d_uav_1 > -0.0001)
                d_uav_1 = T{0.0};
            else
                d_uav_1 = sqrt(arg_d_uav_1);

            T Diff_ = (sb_ - d_uav_1);
            residual[0] = wf_ * 100.0 * log(1.0 + exp(4.0*Diff_) ) ;


            return true;
        }

        double wf_, sb_;
        Grid3d *g_3D_;
        std::unique_ptr<ceres::CostFunctionToFunctor<4,1> > compute_nearest_distance;
    };


private:

};

#endif