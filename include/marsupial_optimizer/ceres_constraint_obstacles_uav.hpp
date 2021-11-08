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
        ObstaclesFunctor(double weight_factor, double safty_bound, 
        pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points)
            : wf_(weight_factor), sb_(safty_bound), kdT_(kdT_From_NN), o_p_(obstacles_Points)
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
            T n_[4], d_, arg_d, Diff_;
            double f_slope_ = 5.0;
            
            (*compute_nearest_distance)(statePos1, n_);
            arg_d = (statePos1[1]-n_[0])*(statePos1[1]-n_[0]) + 
                        (statePos1[2]-n_[1])*(statePos1[2]-n_[1]) + 
                        (statePos1[3]-n_[2])*(statePos1[3]-n_[2]);
            
            if (arg_d < 0.0001 && arg_d > -0.0001)
                d_ = T{0.0};
            else
                d_ = sqrt(arg_d);

            // Diff_ = (sb_*1.2 - d_);
            // if (sb_ < d_)
            //     residual[0] = T{0.0};
			// else
            //     residual[0] = wf_ * 1000.0 * log(1.0 + exp(f_slope_*Diff_) ) ;

            T d_sb_ = sb_ * T{1.05};
            T max_value_residual = T{100.0};
            T min_value_residual = T{0.0};
            T max_value_dependent = T{0.0};
            T m ;
            if (d_ > d_sb_)
                m = T{0.0};
            else
                m = (max_value_residual- min_value_residual)/(max_value_dependent - d_sb_);
            
            residual[0] = wf_ * m *(d_ - d_sb_);

            // std::cout << "ObstacleDistanceFunctorUAV: residual[0]= " << residual[0] << " , d_= " << d_ << std::endl;

            return true;
        }

        double wf_, sb_;
        std::unique_ptr<ceres::CostFunctionToFunctor<4,4> > compute_nearest_distance;
        pcl::KdTreeFLANN <pcl::PointXYZ> kdT_;
        pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_;
    };


private:

};

#endif