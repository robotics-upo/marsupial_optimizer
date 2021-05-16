#ifndef CERES_CONSTRAINS_OBSTACLES_UGV_HPP
#define CERES_CONSTRAINS_OBSTACLES_UGV_HPP

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


class ObstacleDistanceFunctorUGV {

public:
    ObstacleDistanceFunctorUGV(){}
    
    struct ComputeDistanceObstaclesUGV 
    {
        ComputeDistanceObstaclesUGV (pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points)
        : kdT_(kdT_From_NN), o_p_(obstacles_Points)
        {
    
        }

        bool operator()(const double *state1, 
                        double *near_) const 
        {
            NearNeighbor nn;

            nn.nearestObstacleStateCeres(kdT_ , state1[1],state1[2], state1[3], o_p_, near_[0], near_[1], near_[2]);
            // std::cout <<"NearestObstaclesFunctorUGV: state1["<<state1[0]<<"]=[" << state1[1] <<","<<state1[2]<<","<<state1[3] << "] , near=["<<near_[0]<<","<<near_[1]<<","<<near_[2]<<"]"<< std::endl;
            return true;
        }

        double f_, sb_;
        pcl::KdTreeFLANN <pcl::PointXYZ> kdT_;
        pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_;
    };


    struct ObstaclesFunctorUGV 
    {
        ObstaclesFunctorUGV(double weight_factor, double safty_bound, pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points)
            : wf_(weight_factor), sb_(safty_bound), kdT_(kdT_From_NN), o_p_(obstacles_Points)
        {
            compute_nearest_distance.reset(new ceres::CostFunctionToFunctor<4,4>(
                                    new ceres::NumericDiffCostFunction<ComputeDistanceObstaclesUGV,
                                                                        ceres::CENTRAL, 
                                                                        4,
                                                                        4>( 
                                    new ComputeDistanceObstaclesUGV(kdT_, o_p_))));
        }

        template <typename T>
        bool operator()(const T* const statePos1, const T* const statePos2, T* residual) const 
        {
            T d_ugv_1, d_ugv_2;
            T n1_[4];
            T n2_[4];
            (*compute_nearest_distance)(statePos1, n1_);
            (*compute_nearest_distance)(statePos2, n2_);

            d_ugv_1 = sqrt( (statePos1[1]-n1_[0])*(statePos1[1]-n1_[0]) + (statePos1[2]-n1_[1])*(statePos1[2]-n1_[1]) + (statePos1[3]-n1_[2])*(statePos1[3]-n1_[2]));
            d_ugv_2 = sqrt( (statePos2[1]-n2_[0])*(statePos2[1]-n2_[0]) + (statePos2[2]-n2_[1])*(statePos2[2]-n2_[1]) + (statePos2[3]-n2_[2])*(statePos2[3]-n2_[2]));
            // std::cout <<"ObstaclesFunctorUGV d_ugv_=" << d_ugv_ << " , statePos[0] =" << statePos[0] << std::endl;

            // residual[0] =  wf_ * 10.0 * exp(sb_ - d_ugv_);
            residual[0] = wf_ *( exp(4.0*(sb_ - d_ugv_1)) + exp(4.0*(sb_ - d_ugv_2)) + exp(0.5*(d_ugv_1-3.0)) + exp(0.5*(d_ugv_2-3.0)));


            return true;
        }

        std::unique_ptr<ceres::CostFunctionToFunctor<4,4> > compute_nearest_distance;
        double wf_, sb_;
        pcl::KdTreeFLANN <pcl::PointXYZ> kdT_;
        pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_;
    };


private:

};

#endif