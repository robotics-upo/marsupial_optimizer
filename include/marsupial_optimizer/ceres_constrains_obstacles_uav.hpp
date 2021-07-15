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
        ComputeDistanceObstaclesUAV (pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points)
        : kdT_(kdT_From_NN), o_p_(obstacles_Points)
        {
    
        }

        bool operator()(const double *state1, double *near_) const 
        {
            NearNeighbor nn;

            nn.nearestObstacleStateCeres(kdT_ , state1[1],state1[2], state1[3], o_p_, near_[0], near_[1], near_[2]);
            return true;
        }

        double f_, sb_;
        pcl::KdTreeFLANN <pcl::PointXYZ> kdT_;
        pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_;
    };

    struct ObstaclesFunctor 
    {
        ObstaclesFunctor(double weight_factor, double safty_bound, pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points)
            : wf_(weight_factor), sb_(safty_bound), kdT_(kdT_From_NN), o_p_(obstacles_Points)
        {
            compute_nearest_distance.reset(new ceres::CostFunctionToFunctor<4,4>(
                                    new ceres::NumericDiffCostFunction<ComputeDistanceObstaclesUAV,
                                                                        ceres::CENTRAL, 
                                                                        4,
                                                                        4>( 
                                    new ComputeDistanceObstaclesUAV(kdT_, o_p_))));
        }

        template <typename T>
        bool operator()(const T* const statePos1, T* residual) const 
        {
            // To avoid obstacles
            T d_uav_1;
            T arg_d_uav_1;
            T n1_[4];

            (*compute_nearest_distance)(statePos1, n1_);
            arg_d_uav_1 = (statePos1[1]-n1_[0])*(statePos1[1]-n1_[0]) + (statePos1[2]-n1_[1])*(statePos1[2]-n1_[1]) + (statePos1[3]-n1_[2])*(statePos1[3]-n1_[2]);
            
            if (arg_d_uav_1 < 0.0001 && arg_d_uav_1 > -0.0001)
                d_uav_1 = T{0.0};
            else
                d_uav_1 = sqrt(arg_d_uav_1);

            T Diff_ = (sb_ - d_uav_1);
            residual[0] = wf_ * 100.0 * log(1.0 + exp(4.0*Diff_) ) ;

            // if (Diff_ >= 0.0){
                // // residual[0] = wf_ *10.0* (exp(10.0*Diff_) - 1.0) ;
                // residual[0] = wf_ * log(1.0 + exp(4.0*Diff_) ) ;
            // }
            // else
                // residual[0] = T{0.0};

            // printf("ComputeDistanceObstaclesUAV \n");
            // std::cout << "statePos1: " << statePos1[0] << std::endl;
            // std::cout << "d_uav_1: " << d_uav_1 << " , sb_: " << sb_ << std::endl;
            // std::cout << "residual[0]: " << residual[0] << std::endl;

            return true;
        }

        double wf_, sb_;
        std::unique_ptr<ceres::CostFunctionToFunctor<4,4> > compute_nearest_distance;
        pcl::KdTreeFLANN <pcl::PointXYZ> kdT_;
        pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_;

        std::unique_ptr<ceres::CostFunctionToFunctor<4,4,4> > ray_casting;
    };


private:

};

#endif