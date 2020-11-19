#ifndef CERES_CONTRAIN_OBSTACLES_HPP
#define CERES_CONTRAIN_OBSTACLES_HPP


#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include "marsupial_g2o/nearNeighbor.hpp"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;


class ObstacleDistanceFunctor {

public:
  ObstacleDistanceFunctor(double factor, double safty_bound, pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points)
    :f_(factor), sb_(safty_bound), kdT_(kdT_From_NN), o_p_(obstacles_Points) {}
  
    NearNeighbor nn;
	pcl::KdTreeFLANN <pcl::PointXYZ> kdT_;
	pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_;
  
    double f_, sb_;

    template <typename T>
    bool operator()(const T* const pose1, T* residual) const {

    T  d_;
	Eigen::Matrix<T, 3, 1> near_;

    // Eigen::Vector3d pose;
    // pose = Eigen::Vector3d(pose1[0], pose1[1], pose1[2]);
    Eigen::Matrix<T, 3, 1> pose(pose1);

    
    near_ = nn.nearestObstacleVertexCeres(kdT_ , pose, o_p_);
	double n_[3];
    // n_[3] = {near_[0], near_[1], near_[2]}; 	
    d_ = ((pose1[0]-n_[0])*(pose1[0]-n_[0]) + (pose1[1]-n_[1])*(pose1[1]-n_[1]) + (pose1[2]-n_[2])*(pose1[2]-n_[2]));
    
    if (d_ < sb_)
      residual[0] =  f_ * exp(sb_ - 4.0*d_);
    else
      residual[0] = T(0.0);
    return true;
  }

 private:

};


#endif