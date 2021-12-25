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
#include "misc/marker_publisher.h"

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
  CatenaryFunctor(double weight_factor_1, double weight_factor_3, double safty_bound, float min_length_cat, double length_tether_max_,
  				pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points, Grid3d* grid_3D_,
                pcl::KdTreeFLANN <pcl::PointXYZ> trav_kdT, pcl::PointCloud <pcl::PointXYZ>::Ptr trav_pc, geometry_msgs::Vector3 pos_reel_ugv, int size, 
				geometry_msgs::Vector3 fix_pos_ref, ros::NodeHandlePtr nhP_, octomap::OcTree* octotree_full_, bool write_data)
                : wf_1(weight_factor_1), wf_3(weight_factor_3),sb_(safty_bound), m_L_c_(min_length_cat), ltm_(length_tether_max_), kdT_(kdT_From_NN), 
				o_p_(obstacles_Points), g_3D_(grid_3D_),kdT_trav_(trav_kdT), pc_trav_(trav_pc), pos_reel_ugv_(pos_reel_ugv),size_(size), 
				fp_ref_(fix_pos_ref), o_full_(octotree_full_), w_d_(write_data)
    {
		nhP = nhP_;
		catenary_marker_pub_ = nhP_->advertise<visualization_msgs::MarkerArray>("catenary_marker", 1);
		plane_pub_ = nhP_->advertise<visualization_msgs::MarkerArray>("plane_markerArray", 1);
		obs_plane_pub_ = nhP_->advertise<visualization_msgs::MarkerArray>("obsplane_markerArray", 1);
		
		pr_ugv_[0] = pos_reel_ugv_.x;
		pr_ugv_[1] = pos_reel_ugv_.y;
		pr_ugv_[2] = pos_reel_ugv_.z;
    }

    bool operator()(const double* statePosUAV, const double* statePosUGV, const double* stateCat, double* residual) const 
    {
        MarkerPublisher mP_;
		bisectionCatenary bc(nhP);
	    NearNeighbor nn;
		
		visualization_msgs::MarkerArray catenary_marker_, plane_marker_, obs_plane_marker_;
	    std::vector<geometry_msgs::Point> points_catenary;
		std::vector<double> dist_obst_cat; 
		std::vector<int> cat_between_obs, pos_cat_in_coll;
		geometry_msgs::Point min_point_z_cat;
		int pos_in_cat_z_min;
		int first_coll, last_coll;
		first_coll = last_coll = 0;
		bool collision_ = false;
		// For Marker
   		int id_marker_ = statePosUAV[0];		// Related with Marker frame.id

		double pos_init_cat_[3];
		pos_init_cat_[0] = statePosUGV[1]; 
		pos_init_cat_[1] = statePosUGV[2];
		pos_init_cat_[2] = statePosUGV[3] + pr_ugv_[2];

		double dist = sqrt((statePosUAV[1]-pos_init_cat_[0])*(statePosUAV[1]-pos_init_cat_[0]) + 
							(statePosUAV[2]-pos_init_cat_[1])*(statePosUAV[2]-pos_init_cat_[1]) + 
							(statePosUAV[3]-pos_init_cat_[2])*(statePosUAV[3]-pos_init_cat_[2])); 
		double cost_cat_ = 0.0;
		double d_obs_;
		double min_val_proximity_ = 0.015;
		double min_dist_cat_obst = 1000.0;
		double safety_length = m_L_c_;
		
		double Length_, const_collision_; // this value is use to ensure correct evaluation of catenary model
		if (dist < safety_length){
			if (stateCat[1] < safety_length)
				Length_ = safety_length;
			else
				Length_ = stateCat[1];
		}
		else{
			if (stateCat[1] < dist && dist < 4.0)
				Length_ = 1.010 * dist;
			else if (stateCat[1] < dist && dist > 4.0)
				Length_ = 1.005 * dist;
			else
				Length_ = stateCat[1];
		}

		// std::cout << "CatenaryFunctor: node[" << statePosUAV[0] << "]" << "[" << statePosUAV[1] << "," << statePosUAV[2] << "," <<statePosUAV[3]<< "]" <<std::endl;
		points_catenary.clear();  dist_obst_cat.clear(); pos_cat_in_coll.clear(); cat_between_obs.clear(); 
		bc.readDataForCollisionAnalisys(g_3D_, sb_, o_full_, kdT_trav_, pc_trav_);
		bool just_one_axe = bc.configBisection(Length_, pos_init_cat_[0], pos_init_cat_[1], pos_init_cat_[2], statePosUAV[1], statePosUAV[2], statePosUAV[3], true);
		bc.getPointCatenary3D(points_catenary);
		bc.getStatusCollisionCat(dist_obst_cat, pos_cat_in_coll, cat_between_obs, first_coll, last_coll);
		bc.getMinPointZCat(min_point_z_cat, pos_in_cat_z_min);

		geometry_msgs::Point p_, p_ref_;
		p_.x = statePosUAV[1];
		p_.y = statePosUAV[2];
		p_.z = statePosUAV[3];
		p_ref_.x = fp_ref_.x;
        p_ref_.y = fp_ref_.y;
        p_ref_.z = fp_ref_.z;
		// std::cout << "p_ref=[" << p_ref_.x << "," << p_ref_.y << "," << p_ref_.z << "] , p_=[" << p_.x <<","<< p_.y <<"," << p_.z <<"]" << std::endl;

		if (( (p_ref_.x-p_.x) != 0.0 || (p_ref_.y-p_.y) != 0.0 || (p_ref_.z-p_.z) != 0.0) && first_coll!= 0){
			octomap::point3d r_;
			octomap::point3d s_(p_ref_.x, p_ref_.y, p_ref_.z); //start for rayCast
			octomap::point3d d_(p_.x - p_ref_.x , p_.y - p_ref_.y , p_.z - p_ref_.z); //direction for rayCast
			bool r_cast_coll = false; 
			r_cast_coll =  o_full_->castRay(s_, d_, r_);
			double dist_b_uav = sqrt((statePosUAV[1]-s_.x())*(statePosUAV[1]-s_.x())+
									 (statePosUAV[2]-s_.y())*(statePosUAV[2]-s_.y())+ 
									 (statePosUAV[3]-s_.z())*(statePosUAV[3]-s_.z()) );
			double distObs_ = sqrt ((s_.x()-r_.x())*(s_.x()-r_.x())+(s_.y()-r_.y())*(s_.y()-r_.y())+(s_.z()-r_.z())*(s_.z()-r_.z()) );
			if(r_cast_coll && distObs_ <= dist_b_uav )
				last_coll = points_catenary.size();
		}

		mP_.markerPoints(catenary_marker_, points_catenary, id_marker_, size_, catenary_marker_pub_);

		double x_, y_, z_, cost_;
		double min_dis_coll_cat = 0.1;
		double d_below_z_trav = 0.0;
		geometry_msgs::Point n_coll_cat, point_coll_trav;
		for (size_t i = 0; i < points_catenary.size(); i++){
			TrilinearParams d = g_3D_->getPointDistInterpolation((double)points_catenary[i].x, (double)points_catenary[i].y, (double)points_catenary[i].z);
			x_ = points_catenary[i].x;
			y_ = points_catenary[i].y;
			z_ = points_catenary[i].z;
			d_obs_= (d.a0 + d.a1*x_ + d.a2*y_ + d.a3*z_ + d.a4*x_*y_ + d.a5*x_*z_ + d.a6*y_*z_ + d.a7*x_*y_*z_);

			if((i >= first_coll && i <= last_coll && i > 0) ){
				// if (d_obs_ < min_val_proximity_)
				// 	d_obs_ = min_val_proximity_ ;
				// if (d_obs_ < min_dist_cat_obst)
				// 	min_dist_cat_obst = d_obs_;
				cost_ = (1.0/min_val_proximity_)*6.0;
				collision_ = true;
			}
			else if (d_obs_ < sb_) {
				// if (d_obs_ < min_val_proximity_)
				// 	d_obs_ = min_val_proximity_ ;
				cost_ = (1.0/min_val_proximity_)*6.0;
				collision_ = true;
			}
			else{
				// if (d_obs_ < min_val_proximity_)
				// 	d_obs_ = min_val_proximity_ ;
				cost_ = (1.0/d_obs_)*1.0;
			}
			cost_cat_ = cost_cat_ + cost_;
		}

		if (collision_)
			const_collision_ = 0.0;
		else
			const_collision_ = 0.0;

		/********** I Constraint to make cable far away from obstacles **********/
		
		residual[0] = wf_1 * (cost_cat_ + const_collision_);


		// std::cout << std::fixed;
		// std::cout << std::setprecision(8);
		// std::cout << "CatenaryFunctor: node[" << statePosUAV[0] << "] , residual[0]= " <<residual[0]; 
		// std::cout <<"] , [L=" << stateCat[1]<< "/d_s=" << safety_length << "/d=" << dist << "] , points_cat_in_coll=["
		// 		  <<first_coll<<"-"<<last_coll<<"/"<<last_coll-first_coll << "]" <<std::endl;

		if(w_d_){
			std::ofstream ofs;
			std::string name_output_file = "/home/simon/residuals_optimization_data/catenary.txt";
			ofs.open(name_output_file.c_str(), std::ofstream::app);
			if (ofs.is_open()) 
				ofs << residual[0] << " / "
					<< std::endl;
			ofs.close();
			std::ofstream ofs2;
			std::string name_output_file2 = "/home/simon/residuals_optimization_data/catenary2.txt";
			ofs2.open(name_output_file2.c_str(), std::ofstream::app);
			if (ofs2.is_open()) 
				ofs2 << residual[0] << " ; "
					<< "[" << statePosUGV[0] << "] ; "
					<< "L= " << stateCat[1] << " ; "
					<< "safty_d= " << safety_length << " ; "
					<< "first_coll= " << first_coll << " ; "
					<< "last_coll= " << last_coll << " ; "
					<< "n_cat_coll= " << last_coll-first_coll << " ; "
					<< "cat.size()= " << points_catenary.size() << " ; "
					<< "d_below_z_trav= " << d_below_z_trav << " /"
					<<std::endl;
			ofs2.close();
		}
	
		// std::string yy_ ;
		// yy_ = "s";
		// if(statePosUAV[0] == 1 ){
		// 	/********************* To obligate stop method and check Optimization result *********************/
		// 		std::cout << " *** Press key 'y' to continue:  ";
		// 		while (yy_ != "y"){
		// 			std::cin >> yy_ ;
		// 		}
		// 	/**********************************************************************************************/
		// }


	return true;
    }

 	bool w_d_;
    double wf_1, wf_2, wf_3, sb_, ltm_;
	float m_L_c_;
	int size_;
	geometry_msgs::Vector3 pos_reel_ugv_;
    double pr_ugv_[3];
    pcl::KdTreeFLANN <pcl::PointXYZ> kdT_;
    pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_;
	Grid3d* g_3D_;
	pcl::KdTreeFLANN <pcl::PointXYZ> kdT_trav_;
    pcl::PointCloud <pcl::PointXYZ>::Ptr pc_trav_;
	geometry_msgs::Vector3 fp_ref_;
	ros::Publisher catenary_marker_pub_ , plane_pub_, obs_plane_pub_;
	octomap::OcTree* o_full_;
	ros::NodeHandlePtr nhP;

};

#endif