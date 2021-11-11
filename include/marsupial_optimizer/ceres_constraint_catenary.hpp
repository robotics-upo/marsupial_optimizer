#ifndef CERES_CONSTRAINS_CATENARY_HPP
#define CERES_CONSTRAINS_CATENARY_HPP

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include "misc/near_neighbor.hpp"
#include "misc/bisection_catenary_3D.h"
#include "misc/marker_publisher.hpp"

#include <octomap_msgs/Octomap.h> 
#include <octomap/OcTree.h>

#include <iostream>
#include <fstream>
#include <string>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

struct CatenaryFunctor {
  CatenaryFunctor(double weight_factor_1, double weight_factor_2, double weight_factor_3, double safty_bound, double length_tether_max_,
  				pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points, Grid3d* grid_3D_,
                pcl::KdTreeFLANN <pcl::PointXYZ> trav_kdT, pcl::PointCloud <pcl::PointXYZ>::Ptr trav_pc, geometry_msgs::Vector3 pos_reel_ugv, int size, 
				Eigen::Vector3d fix_pos_ref, ros::NodeHandlePtr nhP, octomap::OcTree* octotree_full_, std::vector<double> &vec_l_min)
                : wf_1(weight_factor_1), wf_2(weight_factor_2), wf_3(weight_factor_3),sb_(safty_bound), ltm_(length_tether_max_), kdT_(kdT_From_NN), 
				o_p_(obstacles_Points), g_3D_(grid_3D_),kdT_trav_(trav_kdT), pc_trav_(trav_pc), pos_reel_ugv_(pos_reel_ugv),size_(size), 
				fp_ref_(fix_pos_ref), o_full_(octotree_full_)
    {
		catenary_marker_pub_ = nhP->advertise<visualization_msgs::MarkerArray>("catenary_marker", 1);
		plane_pub_ = nhP->advertise<visualization_msgs::MarkerArray>("plane_markerArray", 1);
		obs_plane_pub_ = nhP->advertise<visualization_msgs::MarkerArray>("obsplane_markerArray", 1);
		pr_ugv_[0] = pos_reel_ugv_.x;
		pr_ugv_[1] = pos_reel_ugv_.y;
		pr_ugv_[2] = pos_reel_ugv_.z;
    }

    bool operator()(const double* statePosUAV, const double* statePosUGV, const double* stateCat, double* residual) const 
    {
        MarkerPublisher mP_;
		bisectionCatenary bc;
		
		visualization_msgs::MarkerArray catenary_marker_, plane_marker_, obs_plane_marker_;
	    std::vector<geometry_msgs::Point> points_catenary;
		std::vector<double> dist_obst_cat; 
		std::vector<int> cat_between_obs, pos_cat_in_coll;
		int first_coll, last_coll;
		// For Marker
   		int id_marker_ = statePosUAV[0];		// Related with Marker frame.id
		// For residual[1] Length Cat
		double max_value_residual = 200.0;
		double min_value_residual = 20.0;

		double pos_init_cat_[3];
		pos_init_cat_[0] = statePosUGV[1]; 
		pos_init_cat_[1] = statePosUGV[2];
		pos_init_cat_[2] = statePosUGV[3] + pr_ugv_[2];

		double dist = sqrt((statePosUAV[1]-pos_init_cat_[0])*(statePosUAV[1]-pos_init_cat_[0]) + 
							(statePosUAV[2]-pos_init_cat_[1])*(statePosUAV[2]-pos_init_cat_[1]) + 
							(statePosUAV[3]-pos_init_cat_[2])*(statePosUAV[3]-pos_init_cat_[2])); 
		double cost_cat_ = 0.0;
		double d_obs_;
		double min_val_proximity_ = 0.0001;
		double safety_length ;
		if (dist < 4.0)
			safety_length = 1.010 * dist;
		else
			safety_length = 1.005 * dist;

		points_catenary.clear();  dist_obst_cat.clear(); pos_cat_in_coll.clear(); cat_between_obs.clear(); 
		bc.readDataForCollisionAnalisys(g_3D_, sb_, o_full_, kdT_trav_, pc_trav_);
		bool just_one_axe = bc.configBisection(stateCat[1], pos_init_cat_[0], pos_init_cat_[1], pos_init_cat_[2], statePosUAV[1], statePosUAV[2], statePosUAV[3], true);
		bc.getPointCatenary3D(points_catenary);
		bc.getStatusCollisionCat(dist_obst_cat, pos_cat_in_coll, cat_between_obs, first_coll, last_coll);

		mP_.markerPoints(catenary_marker_, points_catenary, id_marker_, size_, catenary_marker_pub_);

		double x_, y_, z_, cost_;
		for (size_t i = 0; i < points_catenary.size(); i++){
			TrilinearParams d = g_3D_->getPointDistInterpolation((double)points_catenary[i].x, (double)points_catenary[i].y, (double)points_catenary[i].z);
			x_ = points_catenary[i].x;
			y_ = points_catenary[i].y;
			z_ = points_catenary[i].z;
			d_obs_= (d.a0 + d.a1*x_ + d.a2*y_ + d.a3*z_ + d.a4*x_*y_ + d.a5*x_*z_ + d.a6*y_*z_ + d.a7*x_*y_*z_);
			if(d_obs_ < min_val_proximity_ || (i >= first_coll && i <= last_coll) )
				cost_ = 1.0/min_val_proximity_;
			else
				cost_ = 1.0/d_obs_;
			cost_cat_ = cost_cat_ + cost_;
		}

		
		/********** I Constraint to make cable far away from obstacles **********/
		
		residual[0] = wf_1 * (cost_cat_);

		/********** II Constraint to make length Cable longer than distance between UAV and UGV **********/ 

		double m_, f_;
		if (stateCat[1] < safety_length) {
			m_ = (max_value_residual - min_value_residual)/(safety_length*0.9 - safety_length);
			f_ = 1.0;
		}
		else{
			m_ = 0.0;
			f_ = 0.0;		
		}
		residual[1] = wf_2 *  ( m_ * (stateCat[1] - safety_length) + min_value_residual) * f_;

		/********** III Constraint to make cable not place below traversable map **********/

		residual[2] = wf_3 * 0.0;

		// std::cout << std::fixed;
		// std::cout << std::setprecision(5);
		// std::cout << "CatenaryFunctor: node[" << statePosUAV[0] << "/" << statePosUGV[0] << "]  , residual[0]= " <<residual[0] << " residual[1]= " <<residual[1] << " residual[2]= " <<residual[2]; 
		// std::cout <<"] , [L=" << stateCat[1]<< "/d=" << safety_length <<" , points_cat_in_coll=["
		// 		  <<first_coll<<"-"<<last_coll<<"/"<<last_coll-first_coll << "]"<<std::endl;

		std::ofstream ofs;
		std::string name_output_file = "/home/simon/residuals_optimization_data/catenary.txt";
		ofs.open(name_output_file.c_str(), std::ofstream::app);
		if (ofs.is_open()) 
			ofs << residual[0] << " ; "
				<< residual[1] << " ; "
				<< residual[2] << " ; "
				<<std::endl;
		ofs.close();
	
		// std::string y_ ;
		// y_ = "s";
		// if(statePosUAV[0] == 1 ){
		// 	/********************* To obligate stop method and check Optimization result *********************/
		// 		std::cout << " *** Press key 'y' to continue: " << std::endl;
		// 		while (y_ != "y"){
		// 			std::cin >> y_ ;
		// 		}
		// 	/**********************************************************************************************/
		// }


	return true;
    }

    double wf_1, wf_2, wf_3, sb_, ltm_;
	int size_;
	geometry_msgs::Vector3 pos_reel_ugv_;
    double pr_ugv_[3];
    pcl::KdTreeFLANN <pcl::PointXYZ> kdT_;
    pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_;
	Grid3d* g_3D_;
	pcl::KdTreeFLANN <pcl::PointXYZ> kdT_trav_;
    pcl::PointCloud <pcl::PointXYZ>::Ptr pc_trav_;
	Eigen::Vector3d fp_ref_;
	ros::Publisher catenary_marker_pub_ , plane_pub_, obs_plane_pub_;
	octomap::OcTree* o_full_;
};

#endif