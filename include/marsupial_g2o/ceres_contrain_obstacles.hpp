// #ifndef CERES_CONTRAIN_OBSTACLES_HPP
// #define CERES_CONTRAIN_OBSTACLES_HPP


// #include "ceres/ceres.h"
// #include "glog/logging.h"
// #include "Eigen/Core"

// #include <pcl/search/impl/kdtree.hpp>
// #include <pcl/kdtree/impl/kdtree_flann.hpp>
// #include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/point_types.h>

// #include "marsupial_g2o/nearNeighbor.hpp"

// using ceres::AutoDiffCostFunction;
// using ceres::CostFunction;
// using ceres::Problem;
// using ceres::Solve;
// using ceres::Solver;


// class ObstacleDistanceFunctor {

// public:
//   ObstacleDistanceFunctor(double factor, double safty_bound): f_(factor), sb_(safty_bound) {}
  
// 	NearNeighbor nn;
// 	Eigen::Vector3d near_;
// 	pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN;
// 	pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points;
  
//   double f_, sb_;

//   template <typename T>
//   bool operator()(const T* const pose1, T* residual) const {

//     T  d_;

//     Eigen::Vector3d pose;
//     pose = Eigen::Vector3d(pose1[0],pose1[1],pose1[2]);
//     near_ = nn.nearestObstacleVertex(kdT_From_NN , pose, obstacles_Points);
// 		d_ = (pose -near_).norm();
    
//     if (d_ < sb_)
//       residual[0] =  f_ * exp(sb_ - 4.0*d_);
//     else
//       residual[0] = T(0.0);
//     return true;
//   }

//  private:

// };


// #endif