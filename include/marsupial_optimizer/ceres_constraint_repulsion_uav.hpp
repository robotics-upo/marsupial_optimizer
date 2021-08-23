#ifndef CERES_CONSTRAINS_REPULSION_UAV_HPP
#define CERES_CONSTRAINS_REPULSION_UAV_HPP

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


class RepulsionFunctorUAV {

public:
    RepulsionFunctorUAV(){}
    
    struct ComputeDistanceObstaclesUAV 
    {
        ComputeDistanceObstaclesUAV (pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points)
                                    : kdT_(kdT_From_NN), o_p_(obstacles_Points)
        { }

        bool operator()(const double *point1, double *near_) const 
        {
            NearNeighbor nn;

            nn.nearestObstacleStateCeres(kdT_ , point1[0],point1[1], point1[2], o_p_, near_[0], near_[1], near_[2]);
            return true;
        }

        pcl::KdTreeFLANN <pcl::PointXYZ> kdT_;
        pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_;
    };

    struct RepulsionFunctor 
    {
        RepulsionFunctor(double weight_factor, double dist_fix, pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, 
                         pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points, Eigen::Vector3d fix_pos_ref)
            : wf_(weight_factor), df_(dist_fix), kdT_(kdT_From_NN), o_p_(obstacles_Points), f_p_r_(fix_pos_ref)
        {
            compute_nearest_distance.reset(new ceres::CostFunctionToFunctor<4,4>(
                                    new ceres::NumericDiffCostFunction<ComputeDistanceObstaclesUAV,
                                                                        ceres::FORWARD, 
                                                                        4,
                                                                        4>( 
                                    new ComputeDistanceObstaclesUAV(kdT_, o_p_))));
        }

        template <typename T>
        bool operator()(const T* const statePos1, T* residual) const 
        {
            // To avoid obstacles
            T arg_d_f, arg_d_p, arg_d ;
            T d_f_, d_p_, d_ ;
            T nearest[4];
            T fix_p[4] = {T{f_p_r_.x()},T{f_p_r_.y()},T{f_p_r_.z()},T{0.0}};

            (*compute_nearest_distance)(fix_p, nearest);
            // arg_d_f = pow(fix_p[0]-nearest[0],2) + pow(fix_p[1]-nearest[1],2) + pow(fix_p[2]-nearest[2],2);
            arg_d  =  pow(statePos1[1]-fix_p[0],2) + pow(statePos1[2]-fix_p[1],2) + pow(statePos1[3]-fix_p[2],2);
            arg_d_p = pow(statePos1[1]-nearest[0],2) + pow(statePos1[2]-nearest[1],2) + pow(statePos1[3]-nearest[2],2);
            
            // if (arg_d_f < 0.0001 && arg_d_f > -0.0001)
            //     d_f_ = T{0.0};
            // else
            //     d_f_ = sqrt(arg_d_f);
            d_f_ = T{df_};
            
            if (arg_d < 0.0001 && arg_d > -0.0001)
                d_ = T{0.0}; 
            else
                d_ = sqrt(arg_d);
            if (arg_d_p < 0.0001 && arg_d_p > -0.0001)
                d_p_ = T{0.0};
            else
                d_p_ = sqrt(arg_d_p);

            residual[0] = wf_ * pow((d_f_ + d_) - d_p_ ,2);

            // std::cout <<"["<<statePos1[0] <<"] residual[0]= "<<residual[0]<< " , d_f_=" << d_f_<< " , d_=" << d_<< " , d_p_=" << d_p_<< std::endl;

            return true;
        }

        double wf_, df_;
        std::unique_ptr<ceres::CostFunctionToFunctor<4,4> > compute_nearest_distance;
        pcl::KdTreeFLANN <pcl::PointXYZ> kdT_;
        pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_;
        Eigen::Vector3d f_p_r_;
    };


private:

};

#endif