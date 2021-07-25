#ifndef CERES_CONSTRAINS_CATENARY_HPP
#define CERES_CONSTRAINS_CATENARY_HPP


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
  CatenaryFunctor(double weight_factor_1, double weight_factor_2, double weight_factor_3, double safty_bound, pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points, 
                pcl::KdTreeFLANN <pcl::PointXYZ> trav_kdT, pcl::PointCloud <pcl::PointXYZ>::Ptr trav_pc, geometry_msgs::Point pos_reel_ugv, int size, 
				double pos_reel_z, Eigen::Vector3d fix_pos_ref, ros::NodeHandlePtr nhP)
                : wf_1(weight_factor_1), wf_2(weight_factor_2), wf_3(weight_factor_3),sb_(safty_bound), kdT_(kdT_From_NN), o_p_(obstacles_Points), 
				kdT_trav_(trav_kdT), pc_trav_(trav_pc), pos_reel_ugv_(pos_reel_ugv),s_(size), pr_z_(pos_reel_z), fp_ref_(fix_pos_ref)
    {
		catenary_marker_pub_ = nhP->advertise<visualization_msgs::MarkerArray>("catenary_marker", 1);
		pr_ugv_[0] = pos_reel_ugv_.x;
		pr_ugv_[1] = pos_reel_ugv_.y;
		pr_ugv_[2] = pos_reel_ugv_.z;
    }

    bool operator()(const double* statePosUAV, const double* statePosUGV, const double* stateCat, double* residual) const 
    {
		CatenarySolver cS;
        MarkerPublisher mP_;
	    NearNeighbor nn;

        double d_obs_[1];
        double d_min[1] = {100.0}; //Initial nearest distance catenary to obstacle
		
   		int id_marker_;		// Related with Marker frame.id
        int n_points_cat_dis;

        double d_max_below_z[1] = {0.0};

        visualization_msgs::MarkerArray catenary_marker_;
	    std::vector<geometry_msgs::Point> points_catenary;

		double pos_init_cat_[3];
		pos_init_cat_[0] = statePosUGV[1]; 
		pos_init_cat_[1] = statePosUGV[2];
		pos_init_cat_[2] = statePosUGV[3] + pr_z_;

		double dist_ = sqrt((statePosUAV[1]-pos_init_cat_[0])*(statePosUAV[1]-pos_init_cat_[0]) + 
							(statePosUAV[2]-pos_init_cat_[1])*(statePosUAV[2]-pos_init_cat_[1]) + 
							(statePosUAV[3]-pos_init_cat_[2])*(statePosUAV[3]-pos_init_cat_[2])); 
		double safety_length = 1.02 * dist_;

		points_catenary.clear();
		cS.setMaxNumIterations(100);
		cS.solve(pos_init_cat_[0], pos_init_cat_[1], pos_init_cat_[2], 
				 statePosUAV[1], statePosUAV[2], statePosUAV[3], 
				 stateCat[1], 
				 points_catenary);

		if (points_catenary.size()<1.0)
			ROS_ERROR ("Not posible to get Catenary for state[%f] = [%f %f %f / %f %f %f]", statePosUAV[0], 
																							statePosUGV[1], statePosUGV[2], statePosUGV[3], 
																							statePosUAV[1], statePosUAV[2], statePosUAV[3]);

		id_marker_ = statePosUAV[0];
				
		if (safety_length>= stateCat[1]){
			// ROS_ERROR ("state[%f] ,  Length_Catenary < dist_  ( [%f] < [%f] )",statePosUAV[0], stateCat[1], safety_length);
		}
		else
			mP_.markerPoints(catenary_marker_, points_catenary, id_marker_, s_, catenary_marker_pub_);

		n_points_cat_dis = ceil(1.5*ceil(stateCat[1])); // parameter to ignore collsion points in the begining and in the end of catenary
		if (n_points_cat_dis < 5)
			n_points_cat_dis = 5;

		// int num_point_cat_nearest_coll_= points_catenary.size()-1;
		for (size_t i = 0 ; i < points_catenary.size() ; i++){
		    double near_[3];
			if (i >= n_points_cat_dis && (i < points_catenary.size() - n_points_cat_dis/2.0) ){
				nn.nearestObstacleStateCeres(kdT_ , points_catenary[i].x, points_catenary[i].y, points_catenary[i].z, o_p_, near_[0], near_[1], near_[2]);
				d_obs_[0] = sqrt((points_catenary[i].x-near_[0])*(points_catenary[i].x-near_[0])+
								 (points_catenary[i].y-near_[1])*(points_catenary[i].y-near_[1])+(points_catenary[i].z-near_[2])*(points_catenary[i].z-near_[2]));
				if (d_obs_[0] < d_min[0]){
					d_min[0] = d_obs_[0];
					// num_point_cat_nearest_coll_ = i;
				}
			}
		    double below_z[3];
		    double d_below_z[1];
			nn.nearestObstacleStateCeres(kdT_trav_ , points_catenary[i].x, points_catenary[i].y, points_catenary[i].z, pc_trav_, below_z[0], below_z[1], below_z[2]);
			if (points_catenary[i].z <= below_z[2]){
				d_below_z[0] = sqrt((points_catenary[i].x-below_z[0])*(points_catenary[i].x-below_z[0])+(points_catenary[i].y-below_z[1])*(points_catenary[i].y- below_z[1])+(points_catenary[i].z-below_z[2])*(points_catenary[i].z-below_z[2]));
				if (d_below_z[0] > d_max_below_z[0])
					d_max_below_z[0] = d_below_z[0]; 
			}
		}
		double dist_uav_ref = sqrt( pow(fp_ref_.x()-statePosUAV[1],2) + pow(fp_ref_.y()-statePosUAV[2],2) + pow(fp_ref_.z()-statePosUAV[3],2));
		// double num_points_cat_above_obs_  = ((double)points_catenary.size() - (double)num_point_cat_nearest_coll_);

		if (d_min[0] > sb_)
			residual[0] = 0.0;
		else	
			residual[0] = wf_1 * 100.0 * (10.0*dist_uav_ref) * (exp(10.0*(sb_-d_min[0]) ) - 1.0);
			// residual[0] = wf_ * 10.0 * num_points_cat_above_obs_ * (exp(10.0*(d_min[0]-sb_) ) - 1.0);
		// Constraint to make length Cable longer than 
		if (stateCat[1] > safety_length)
			residual[1] = 0.0;
		else
			residual[1] = wf_2 *100.0 * (exp(10.0*(safety_length - stateCat[1])) - 1.0);

		residual[2] = wf_3 * 1000.0 * (exp(10.0* d_max_below_z[0]) - 1.0);


		std::cout << "residual[0] = " <<residual[0] << " , dist_uav_ref= " << dist_uav_ref << " , d_min[0]= " << d_min[0] << " , sb_= " << sb_ << std::endl;
		std::cout << "residual[1] = " <<residual[1] << " , ["<< stateCat[0] << "]stateCat[1]= " << stateCat[1] << " , safety_length= " << safety_length << std::endl;
		std::cout << "residual[2] = " <<residual[2] << " , d_max_below_z[0]= " << d_max_below_z[0] << std::endl;

		return true;
    }

    double wf_1, wf_2, wf_3, sb_, pr_z_;
	int s_;
	geometry_msgs::Point pos_reel_ugv_;
    double pr_ugv_[3];
    pcl::KdTreeFLANN <pcl::PointXYZ> kdT_;
    pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_;

	pcl::KdTreeFLANN <pcl::PointXYZ> kdT_trav_;
    pcl::PointCloud <pcl::PointXYZ>::Ptr pc_trav_;
	Eigen::Vector3d fp_ref_;

	ros::Publisher catenary_marker_pub_;
};

#endif