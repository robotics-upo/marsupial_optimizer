#ifndef CERES_CONTRAIN_OBSTACLES_TEST_HPP
#define CERES_CONTRAIN_OBSTACLES_TEST_HPP


#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include "marsupial_g2o/near_neighbor.hpp"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;



class ObstacleDistanceFunctor {

public:
    ObstacleDistanceFunctor(){}
    
    struct ComputeObstacleFunctor {
        ComputeObstacleFunctor (double factor, double safty_bound, pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points)
        : f_(factor), sb_(safty_bound), kdT_(kdT_From_NN), o_p_(obstacles_Points)
        {
    
        }

        bool operator()(const double *pose, 
                        double *near_) const 
        {
            NearNeighbor nn;
            nn.nearestObstacleVertexCeres(kdT_ , pose[0],pose[1], pose[2], o_p_, near_[0], near_[1], near_[2]);
            return true;
        }

        double f_, sb_;
        pcl::KdTreeFLANN <pcl::PointXYZ> kdT_;
        pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_;
    };

    struct Affine4DWithDistortion {
        Affine4DWithDistortion(double factor, double safty_bound, pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points)
            : f_(factor), sb_(safty_bound), kdT_(kdT_From_NN), o_p_(obstacles_Points)
        {
            compute_distortion.reset(new ceres::CostFunctionToFunctor<4,4>(
                                    new ceres::NumericDiffCostFunction<ComputeObstacleFunctor,
                                                                        ceres::CENTRAL, 
                                                                        4,
                                                                        4>( 
                                    new ComputeObstacleFunctor(f_, sb_, kdT_, o_p_))));
        }

        template <typename T>
        bool operator()(const T* const pose1, 
                        T* residual) const 
        {
            T d_;
            T n_[4];
            (*compute_distortion)(pose1, n_);

            d_ = ((pose1[0]-n_[0])*(pose1[0]-n_[0]) + (pose1[1]-n_[1])*(pose1[1]-n_[1]) + (pose1[2]-n_[2])*(pose1[2]-n_[2]));
            if (d_ < sb_)
                residual[0] =  f_ * exp(sb_ - 4.0*d_);
            else
                residual[0] = T(0.0);
            return true;
        }

        std::unique_ptr<ceres::CostFunctionToFunctor<4,4> > compute_distortion;
        double f_, sb_;
        pcl::KdTreeFLANN <pcl::PointXYZ> kdT_;
        pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_;
    };

private:

};


#endif
