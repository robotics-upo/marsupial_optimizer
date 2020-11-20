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
  
	pcl::KdTreeFLANN <pcl::PointXYZ> kdT_;
	pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_;
    
    double f_, sb_;


struct ObstacleFunctor {
  bool operator()(const double* pose, double* near_) const {
    NearNeighbor nn;
    nn.nearestObstacleVertexCeres(kdT_ , pose[0],pose[1], pose[2], OF.o_p_, near_[0], near_[1], near_[2]);
    return true;
  }
};

struct Affine4DWithDistortion {
  Affine4DWithDistortion(const double x_in[4], const double y_in[4]) {
    x[0] = x_in[0];
    x[1] = x_in[1];
    x[2] = x_in[2];
    x[3] = x_in[3];
    y[0] = y_in[0];
    y[1] = y_in[1];
    y[2] = y_in[2];
    y[3] = y_in[3];

    compute_distortion.reset(new ceres::CostFunctionToFunctor<4, 4>(
         new ceres::NumericDiffCostFunction<ObstacleFunctor,
                                            ceres::CENTRAL,
                                            4,
                                            4>(
            new ObstacleFunctor)));
  }

  template <typename T>
  bool operator()(const T* const pose1, T* residual) const {
    T d_;
    T n_;

    (*compute_distortion)(&pose1, &n_,kdT_, o_p_);

    // T n_[3] = {near_[0], near_[1], near_[2]}; 	
    d_ = ((pose1[0]-n_[0])*(pose1[0]-n_[0]) + (pose1[1]-n_[1])*(pose1[1]-n_[1]) + (pose1[2]-n_[2])*(pose1[2]-n_[2]));

    if (d_ < sb_)
      residual[0] =  f_ * exp(sb_ - 4.0*d_);
    else
      residual[0] = T(0.0);
    return true;
  }

  double x[4];
  double y[4];
  std::unique_ptr<ceres::CostFunctionToFunctor<4, 4> > compute_distortion;
};

 private:

};


#endif