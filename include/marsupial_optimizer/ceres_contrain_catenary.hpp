#ifndef CERES_CONTRAIN_CATENARY_HPP
#define CERES_CONTRAIN_CATENARY_HPP


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

#include "misc/near_neighbor.hpp"
#include "misc/marker_publisher.hpp"
#include "misc/catenary_solver_ceres.hpp"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;


struct CatenaryFunctor {
  CatenaryFunctor(double weight_factor, double safty_bound, double initial_length_catenary,pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points, 
                 geometry_msgs::Point pos_reel_ugv, int size, ros::NodeHandlePtr nhP)
                : wf_(weight_factor), sb_(safty_bound), ilc_(initial_length_catenary), kdT_(kdT_From_NN), o_p_(obstacles_Points), pos_reel_ugv_(pos_reel_ugv),s_(size)
    {
		catenary_marker_pub_ = nhP->advertise<visualization_msgs::MarkerArray>("catenary_marker", 1);
		pr_ugv_[0] = pos_reel_ugv_.x;
		pr_ugv_[1] = pos_reel_ugv_.y;
		pr_ugv_[2] = pos_reel_ugv_.z;
    }

    bool operator()(const double* statePos, const double* stateRot, const double* stateCat, double* residual) const 
    {
		CatenarySolver cS;
        MarkerPublisher mP_;
	    NearNeighbor nn;

        double d_[1];
        double d_min[1] = {100.0};
		
   		int id_marker_;		// Related with Marker frame.id
        int n_points_cat_dis;

        double lower_point_cat_z;

        visualization_msgs::MarkerArray catenary_marker_;
	    std::vector<geometry_msgs::Point> points_catenary;

		//Get UV Reel position
		double roll_, pitch_, yaw_, lengt_vec_;
		tf::Quaternion q_(stateRot[1],stateRot[2],stateRot[3],stateRot[4]);
		tf::Matrix3x3 M_(q_);	
		M_.getRPY(roll_, pitch_, yaw_);
		lengt_vec_ =  sqrt(pr_ugv_[0]*pr_ugv_[0] + pr_ugv_[1]*pr_ugv_[1]);
		double pos_init_cat_[3];
		pos_init_cat_[0] = statePos[1] + lengt_vec_ *cos(yaw_); 
		pos_init_cat_[1] = statePos[2] + lengt_vec_ *sin(yaw_);
		pos_init_cat_[2] = statePos[3] + pr_ugv_[2] ;

		double dist_ = sqrt((statePos[4]-pos_init_cat_[0])*(statePos[4]-pos_init_cat_[0]) + 
							(statePos[5]-pos_init_cat_[1])*(statePos[5]-pos_init_cat_[1]) + 
							(statePos[6]-pos_init_cat_[2])*(statePos[6]-pos_init_cat_[2])); 
		double safety_length = 1.001 * dist_;

		points_catenary.clear();
		cS.setMaxNumIterations(100);
		cS.solve(pos_init_cat_[0], pos_init_cat_[1], pos_init_cat_[2], statePos[4], statePos[5], statePos[6], stateCat[1], points_catenary);

		if (points_catenary.size()<1.0)
			ROS_ERROR ("Not posible to get Catenary for state[%f] = [%f %f %f / %f %f %f]", statePos[0],statePos[1], statePos[2], statePos[3], statePos[4], statePos[5], statePos[6]);

		id_marker_ = statePos[0];
				
		if (safety_length>= stateCat[1]){
			// ROS_ERROR ("state[%f] ,  Length_Catenary < dist_  ( [%f] < [%f] )",statePos[0], stateCat[1], safety_length);
		}
		else
			mP_.markerPoints(catenary_marker_, points_catenary, id_marker_, s_, catenary_marker_pub_);

		n_points_cat_dis = ceil(1.5*ceil(stateCat[1])); // parameter to ignore collsion points in the begining and in the end of catenary
		if (n_points_cat_dis < 5)
			n_points_cat_dis = 5;

		// size_t _num_pos_nearest_point_cat;
		// double _point_cat_nearest_obs[3];

		for (size_t i = 0 ; i < points_catenary.size() ; i++){
		       double near_[3];
			if (i >= n_points_cat_dis && (i < points_catenary.size() - n_points_cat_dis/2.0) ){
				nn.nearestObstacleStateCeres(kdT_ , points_catenary[i].x, points_catenary[i].y, points_catenary[i].z, o_p_, near_[0], near_[1], near_[2]);
				d_[0] = sqrt((points_catenary[i].x-near_[0])*(points_catenary[i].x-near_[0])+(points_catenary[i].y-near_[1])*(points_catenary[i].y- near_[1])+(points_catenary[i].z-near_[2])*(points_catenary[i].z-near_[2]));
				if (d_[0] < d_min[0]){
					d_min[0] = d_[0];
					// _num_pos_nearest_point_cat = i;
					// _point_cat_nearest_obs[0] = points_catenary[i].x; 
					// _point_cat_nearest_obs[1] = points_catenary[i].y; 
					// _point_cat_nearest_obs[2] = points_catenary[i].z;
				}
			}
		}

		
		residual[0] = wf_ * 4.0 /(1.0 + exp(40.0*(d_min[0]-sb_)));
		residual[1] = wf_ * 4.0 /(1.0 + exp(40.0*(stateCat[1]- safety_length)));

		return true;
    }

    double wf_, sb_, ilc_;
	int s_;
	geometry_msgs::Point pos_reel_ugv_;
    double pr_ugv_[3];
    pcl::KdTreeFLANN <pcl::PointXYZ> kdT_;
    pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_;

	ros::Publisher catenary_marker_pub_;
};

#endif