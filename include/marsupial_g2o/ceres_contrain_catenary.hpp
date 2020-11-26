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

#include "marsupial_g2o/near_neighbor.hpp"
#include "marsupial_g2o/bisection_catenary_3D.h"
#include "marsupial_g2o/marker_publisher.hpp"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;


struct CatenaryFunctor {
  CatenaryFunctor(double weight_factor, double safty_bound, double initial_length_catenary,pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points, double z_constraint, 
                double bound_bisection_a, double bound_bisection_b, geometry_msgs::Point ugv_pos_catenary, double size)
                : wf_(weight_factor), sb_(safty_bound), ilc_(initial_length_catenary), kdT_(kdT_From_NN), o_p_(obstacles_Points), z_c_(z_constraint), bba_(bound_bisection_a), bbb_(bound_bisection_b), ugv_pc_(ugv_pos_catenary), s_(size)
    {
    }

    bool operator()(const double* state1, double* residual) const 
    {
        NearNeighbor nn_;
        bisectionCatenary bC_;
        MarkerPublisher mP_;
        double d_[1];
        double near_[4];

   		int id_marker_;		// Related with Marker frame.id
        int n_vertices_;	// Number of Vertices that no are fixed
        int prev_size_marker_ = 0; // Save the previus size value of the catenary for vertex optimized To delete marker 
        int n_points_cat_dis;
        double factor_error_straight_catenary;
        double sum_error_per_cat;

        double bound_z_negative;
        double lower_point_cat_z;

        visualization_msgs::MarkerArray catenary_marker_;
	    std::vector<geometry_msgs::Point> points_catenary;

        double dist_ = sqrt((state1[0]-ugv_pc_.x)*(state1[0]-ugv_pc_.x) + (state1[1]-ugv_pc_.y)*(state1[1]-ugv_pc_.y) + (state1[2]-ugv_pc_.z)*(state1[2]-ugv_pc_.z)); 
        bC_.setNumberPointsCatenary(state1[4]*10.0);
		bC_.setFactorBisection(bba_,bbb_);
		bC_.configBisection(state1[4],ugv_pc_.x,ugv_pc_.y,ugv_pc_.z,state1[0],state1[1],state1[2],1,"state");

		points_catenary.clear();
		bC_.getPointCatenary3D(points_catenary);

		if (points_catenary.size()<1.0){
			// ROS_ERROR ("Not posible to get Catenary for vertex[%i] = [%f %f %f]", state->id(),state->estimate().x(),state->estimate().y(),state->estimate().z());
		}

        id_marker_ = 1;
		if (prev_size_marker_ >= points_catenary.size() )
			mP_.clearMarkers(catenary_marker_, prev_size_marker_, catenary_marker_pub);
		else
			mP_.clearMarkers(catenary_marker_, points_catenary.size(), catenary_marker_pub);
			
		mP_.markerPoints(catenary_marker_, points_catenary, id_marker_, n_vertices_, catenary_marker_pub);
		prev_size_marker_ = points_catenary.size();

		int count_negative_z = 0;

		sum_error_per_cat = 0.0;
		factor_error_straight_catenary = 1.0;

		double factor_negative_z = 0.0;

		n_points_cat_dis = ceil(1.5*ceil(state1[4])); // parameter to ignore collsion points in the begining and in the end of catenary
		if (n_points_cat_dis < 5)
			n_points_cat_dis = 5;

		int n_collision_cat = 0;

        Eigen::Vector3d p_cat;
		for (size_t i = 0 ; i < points_catenary.size() ; i++){
			p_cat.x() = points_catenary[i].x;
			p_cat.y() = points_catenary[i].y;
			p_cat.z() = points_catenary[i].z;
					
            nn_.nearestObstacleVertexCeres(kdT_ , state1[0],state1[1], state1[2], o_p_, near_[0], near_[1], near_[2]);
            d_[0] = ((state1[0]-near_[0])*(state1[0]-near_[0]) + (state1[1]-near_[1])*(state1[1]-near_[1]) + (state1[2]-near_[2])*(state1[2]-near_[2]));

			if (d_[0] < sb_ && (i > n_points_cat_dis ) && (i < points_catenary.size()-n_points_cat_dis/2)){
				sum_error_per_cat = 100000.0 *exp(10.0*(dist_-state1[4]));
				factor_error_straight_catenary = 0.0;
				n_collision_cat ++;
			}

			if (p_cat.z() < bound_z_negative){
				count_negative_z++;
				factor_error_straight_catenary = 1.0;
				factor_negative_z = 1.0;
				if ( p_cat.z() < lower_point_cat_z)
					lower_point_cat_z = p_cat.z();
			}
		}


        residual[0] =  sum_error_per_cat;
		residual[1] = 6.0*(state1[4] -  dist_*1.002)*factor_error_straight_catenary; 
		residual[2] = 10000000.0*state1[4]*exp(10.0*(bound_z_negative + sqrt(pow(bound_z_negative - p_cat.z(),2))) ) * factor_negative_z;

        return true;
    }



    double wf_, sb_, ilc_, z_c_, bba_, bbb_, s_;
    geometry_msgs::Point ugv_pc_;
    pcl::KdTreeFLANN <pcl::PointXYZ> kdT_;
    pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_;

	ros::Publisher catenary_marker_pub;

};




#endif