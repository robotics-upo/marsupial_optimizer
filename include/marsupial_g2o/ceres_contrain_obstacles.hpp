#ifndef CERES_CONTRAIN_OBSTACLES_HPP
#define CERES_CONTRAIN_OBSTACLES_HPP


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


// struct ObstacleDistanceFunctor {
//   ObstacleDistanceFunctor(double weight_factor, double safty_bound, pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points)
//         : wf_(weight_factor), sb_(safty_bound), kdT_(kdT_From_NN), o_p_(obstacles_Points)
//     {
//     }

//     bool operator()(const double* state1, double* residual) const 
//     {
//         double d_[1];
//         NearNeighbor nn;
//         double near_[3];
        
//         nn.nearestObstacleVertexCeres(kdT_ , state1[0],state1[1], state1[2], o_p_, near_[0], near_[1], near_[2]);
//         d_[0] = ((state1[0]-near_[0])*(state1[0]-near_[0]) + (state1[1]-near_[1])*(state1[1]-near_[1]) + (state1[2]-near_[2])*(state1[2]-near_[2]));
//         residual[0] =  wf_ * exp(sb_ - 4.0*d_[0]);

//         return true;
//     }

//     double wf_, sb_;
//     pcl::KdTreeFLANN <pcl::PointXYZ> kdT_;
//     pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_;
// };


class ObstacleDistanceFunctor {

public:
    ObstacleDistanceFunctor(){}
    
    struct ComputeDistanceObstacles {
        ComputeDistanceObstacles (pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points)
        : kdT_(kdT_From_NN), o_p_(obstacles_Points)
        {
    
        }

        bool operator()(const double *state1, 
                        double *near_) const 
        {
            NearNeighbor nn;

            // double _near_[3];
	        // nn.nearestObstacleVertexCeres( kdT_, -3.137500, -0.200000, 2.563860, o_p_, _near_[0], _near_[1], _near_[2]);
            // printf("============================================== \n");
            // printf("OBSTACLE testing kdtree: point = [-3.137500, -0.200000, 2.563860] , near = [%f %f %f] \n",_near_[0], _near_[1], _near_[2]);
            // printf("size obs_points = [%lu]",o_p_->size());
            // printf("============================================== \n");

            nn.nearestObstacleVertexCeres(kdT_ , state1[0],state1[1], state1[2], o_p_, near_[0], near_[1], near_[2]);
            return true;
        }

        double f_, sb_;
        pcl::KdTreeFLANN <pcl::PointXYZ> kdT_;
        pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_;
    };

    struct ObstaclesFunctor {
        ObstaclesFunctor(double weight_factor, double safty_bound, pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points)
            : wf_(weight_factor), sb_(safty_bound), kdT_(kdT_From_NN), o_p_(obstacles_Points)
        {
            compute_distortion.reset(new ceres::CostFunctionToFunctor<4,4>(
                                    new ceres::NumericDiffCostFunction<ComputeDistanceObstacles,
                                                                        ceres::CENTRAL, 
                                                                        4,
                                                                        4>( 
                                    new ComputeDistanceObstacles(kdT_, o_p_))));
        }

        template <typename T>
        bool operator()(const T* const pose1, 
                        T* residual) const 
        {
            T d_;
            T n_[4];
            (*compute_distortion)(pose1, n_);

            d_ = ((pose1[0]-n_[0])*(pose1[0]-n_[0]) + (pose1[1]-n_[1])*(pose1[1]-n_[1]) + (pose1[2]-n_[2])*(pose1[2]-n_[2]));

            residual[0] =  wf_ * exp(sb_ - 4.0*d_);

            return true;
        }

        std::unique_ptr<ceres::CostFunctionToFunctor<4,4> > compute_distortion;
        double wf_, sb_;
        pcl::KdTreeFLANN <pcl::PointXYZ> kdT_;
        pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_;
    };


private:

};

#endif