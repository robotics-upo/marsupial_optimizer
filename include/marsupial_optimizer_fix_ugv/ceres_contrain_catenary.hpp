#ifndef CERES_CONTRAIN_CATENARY_FIX_UGV_HPP
#define CERES_CONTRAIN_CATENARY_FIX_UGV_HPP


#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include "catenary_checker/near_neighbor.hpp"
#include "misc/marker_publisher.h"
#include "misc/catenary_solver_ceres.hpp"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;


struct CatenaryFunctor {
  CatenaryFunctor(double weight_factor, double safty_bound, double initial_length_catenary,
                  pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points,
                  geometry_msgs::Point ugv_pos_catenary, int size, ros::NodeHandlePtr nhP)
                : wf_(weight_factor), sb_(safty_bound), ilc_(initial_length_catenary), kdT_(kdT_From_NN),
                  o_p_(obstacles_Points), ugv_pc_(ugv_pos_catenary), s_(size)
    {
		catenary_marker_pub_ = nhP->advertise<visualization_msgs::MarkerArray>("catenary_marker", 1);
    }

    bool operator()(const double* state1, const double* state2, double* residual) const 
    {
      CatenarySolver cS;
      MarkerPublisher mP_;
	    NearNeighbor nn;

      double d_[1];
      double d_min[1] = {40.0};

   		int id_marker_;		// Related with Marker frame.id
      int n_points_cat_dis;

      double lower_point_cat_z;
      
      visualization_msgs::MarkerArray catenary_marker_;
	    std::vector<geometry_msgs::Vector3> points_catenary;

      double dist_ = sqrt((state1[1] - ugv_pc_.x)*(state1[1] - ugv_pc_.x) +
                          (state1[2] - ugv_pc_.y)*(state1[2] - ugv_pc_.y) +
                          (state1[3] - ugv_pc_.z)*(state1[3] - ugv_pc_.z));
      double safety_length = 1.001 * dist_;

      points_catenary.clear();

      cS.setMaxNumIterations(100);
      cS.solve(ugv_pc_.x, ugv_pc_.y, ugv_pc_.z, state1[1], state1[2], state1[3], state2[1], points_catenary);

      if (points_catenary.size()<1.0)
        ROS_ERROR ("Not posible to get Catenary for state[%f] = [%f %f %f]", state1[1],state1[1], state1[2], state1[3]);

      id_marker_ = state1[0];
				
      if (safety_length>= state2[1])
        ROS_ERROR ("state[%f] ,  Length_Catenary < dist_  ( [%f] < [%f] )",state1[0], state2[1], safety_length);
      else
        mP_.markerPoints(catenary_marker_, points_catenary, id_marker_, s_, catenary_marker_pub_);

      n_points_cat_dis = ceil(1.5*ceil(state2[1])); // ignore collsion points in the begining and in the end of catenary
      if (n_points_cat_dis < 5)
        n_points_cat_dis = 5;

      size_t _num_pos_nearest_point_cat;
      double _point_cat_nearest_obs[3];

      for (size_t i = 0 ; i < points_catenary.size() ; i++){
        double near_[3];
        if (i >= n_points_cat_dis && (i < points_catenary.size() - n_points_cat_dis/2.0) ){
          nn.nearestObstacleStateCeres(kdT_ , points_catenary[i].x, points_catenary[i].y, points_catenary[i].z,
                                       o_p_, near_[0], near_[1], near_[2]);
          d_[0] = sqrt((points_catenary[i].x-near_[0])*(points_catenary[i].x-near_[0]) +
                       (points_catenary[i].y-near_[1])*(points_catenary[i].y-near_[1]) +
                       (points_catenary[i].z-near_[2])*(points_catenary[i].z-near_[2]));
          if (d_[0] < d_min[0]){
            d_min[0] = d_[0];
            _num_pos_nearest_point_cat = i;
            _point_cat_nearest_obs[0] = points_catenary[i].x;
            _point_cat_nearest_obs[1] = points_catenary[i].y;
            _point_cat_nearest_obs[2] = points_catenary[i].z;
          }
        }
      }
      residual[0] = wf_ * 4.0 /(1.0 + exp(40.0*(d_min[0]-sb_)));
      residual[1] = wf_ * 4.0 /(1.0 + exp(40.0*(state2[1]- safety_length)));
      return true;
    }

  double wf_, sb_, ilc_;
	int s_;
  geometry_msgs::Point ugv_pc_;
  pcl::KdTreeFLANN <pcl::PointXYZ> kdT_;
  pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_;
	ros::Publisher catenary_marker_pub_;
};

#endif
