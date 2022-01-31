#ifndef CERES_CONSTRAINS_CATENARY_OBSTACLE_NUMERIC_HPP
#define CERES_CONSTRAINS_CATENARY_OBSTACLE_NUMERIC_HPP

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

// using ceres::AutoDiffCostFunction;
using ceres::NumericDiffCostFunction;
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

    bool operator()(const double* stateUGV, const double* stateUAV, const double* stateCat, double* residual) const 
    {
        MarkerPublisher mP_;
		bisectionCatenary bc(nhP);
	    NearNeighbor nn;
		
		visualization_msgs::MarkerArray catenary_marker_, plane_marker_, obs_plane_marker_;
	    std::vector<geometry_msgs::Point> points_catenary;
		std::vector<double> dist_obst_cat; 
		std::vector<int> cat_between_obs, pos_cat_in_coll;
		geometry_msgs::Point min_point_z_cat, pos_reel;
		int pos_in_cat_z_min, first_coll, last_coll;
		first_coll = last_coll = 0;
		// For Marker
		double max_value_residual = 10000.0;
		double min_value_residual = 1000.0;
   		int id_marker_ = stateUAV[0];		// Related with Marker frame.id

		double Length_, d_obs_; // this value is use to ensure correct evaluation of catenary model
		pos_reel.x = stateUGV[1]; 
		pos_reel.y = stateUGV[2];
		pos_reel.z = stateUGV[3] + pr_ugv_[2];

		double dist = sqrt((stateUAV[1]-pos_reel.x)*(stateUAV[1]-pos_reel.x) + 
						   (stateUAV[2]-pos_reel.y)*(stateUAV[2]-pos_reel.y) + 
						   (stateUAV[3]-pos_reel.z)*(stateUAV[3]-pos_reel.z)); 
		double cost_cat_ = 0.0;
		double min_val_proximity_ = 0.015;
		double safety_length = m_L_c_;
		bool is_LoS = true;

		// Get LoS between UAV and UGV , if It is not LoS is use L_i, in other case distance euclidian as L
		if ((pos_reel.x-stateUAV[1]) != 0.0 || (pos_reel.y-stateUAV[2]) != 0.0 || (pos_reel.z-stateUAV[3]) != 0.0){
			octomap::point3d r_;
			octomap::point3d s_(pos_reel.x, pos_reel.y, pos_reel.z); //start for rayCast
			octomap::point3d d_(stateUAV[1] - pos_reel.x , stateUAV[2] - pos_reel.y , stateUAV[3] - pos_reel.z); //direction for rayCast
			bool r_cast_coll =  o_full_->castRay(s_, d_, r_);
			double dist_ = sqrt(pow(stateUAV[1]-s_.x(),2)+pow(stateUAV[2]-s_.y(),2)+pow(stateUAV[3]-s_.z(),2));
			double distObs_ = sqrt ((s_.x()-r_.x())*(s_.x()-r_.x())+(s_.y()-r_.y())*(s_.y()-r_.y())+(s_.z()-r_.z())*(s_.z()-r_.z()) );
			if(r_cast_coll && distObs_ <= dist_)
				is_LoS = false;
		}

		if (!is_LoS && safety_length > dist){
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

		points_catenary.clear();  dist_obst_cat.clear(); pos_cat_in_coll.clear(); cat_between_obs.clear(); 
		bc.readDataForCollisionAnalisys(g_3D_, sb_, o_full_, kdT_trav_, pc_trav_);
		bool just_one_axe = bc.configBisection(Length_, pos_reel.x, pos_reel.y, pos_reel.z, stateUAV[1], stateUAV[2], stateUAV[3], true);
		bc.getPointCatenary3D(points_catenary);
		bc.getStatusCollisionCat(dist_obst_cat, pos_cat_in_coll, cat_between_obs, first_coll, last_coll);
		bc.getMinPointZCat(min_point_z_cat, pos_in_cat_z_min);

		if (( (fp_ref_.x-stateUAV[1]) != 0.0 || (fp_ref_.y-stateUAV[2]) != 0.0 || (fp_ref_.z-stateUAV[3]) != 0.0) && first_coll!= 0){
			octomap::point3d r_;
			octomap::point3d s_(fp_ref_.x, fp_ref_.y, fp_ref_.z); //start for rayCast
			octomap::point3d d_(stateUAV[1] - fp_ref_.x , stateUAV[2] - fp_ref_.y , stateUAV[3] - fp_ref_.z); //direction for rayCast
			bool r_cast_coll = false; 
			r_cast_coll =  o_full_->castRay(s_, d_, r_);
			double dist_b_uav = sqrt(pow(stateUAV[1]-s_.x(),2)+pow(stateUAV[2]-s_.y(),2)+pow(stateUAV[3]-s_.z(),2));
			double distObs_ = sqrt ((s_.x()-r_.x())*(s_.x()-r_.x())+(s_.y()-r_.y())*(s_.y()-r_.y())+(s_.z()-r_.z())*(s_.z()-r_.z()) );
			if(r_cast_coll && distObs_ <= dist_b_uav )
				last_coll = points_catenary.size();
		}

		mP_.markerPoints(catenary_marker_, points_catenary, id_marker_, size_, catenary_marker_pub_);

		double x_, y_, z_, cost_;
		double d_below_z_trav = 0.0;
		geometry_msgs::Point n_coll_cat, point_coll_trav;
		for (size_t i = 0; i < points_catenary.size(); i++){
			TrilinearParams d = g_3D_->getPointDistInterpolation((double)points_catenary[i].x, (double)points_catenary[i].y, (double)points_catenary[i].z);
			x_ = points_catenary[i].x;
			y_ = points_catenary[i].y;
			z_ = points_catenary[i].z;
			d_obs_= (d.a0 + d.a1*x_ + d.a2*y_ + d.a3*z_ + d.a4*x_*y_ + d.a5*x_*z_ + d.a6*y_*z_ + d.a7*x_*y_*z_);

			if((i >= first_coll && i <= last_coll && i > 0) )
				cost_ = (1.0/min_val_proximity_)*2.0;
			else if (d_obs_ < sb_) 
				cost_ = (1.0/min_val_proximity_)*2.0;
			else
				cost_ = (1.0/d_obs_)*1.0;
			cost_cat_ = cost_cat_ + cost_;
		}

		double m_, f_;
		if (stateCat[1] < Length_) {
			m_ = (max_value_residual - min_value_residual)/(Length_*0.9 - Length_);
			f_ = 1.0;
		}
		else{
			m_ = 0.0;
			f_ = 0.0;		
		}

		/********** I Constraint to make cable far away from obstacles **********/
		double diff_; 
		if (last_coll == 0 && first_coll== 0)
			diff_ = 1.0;
		else
			diff_ = last_coll-first_coll;

		double residual_0 = (cost_cat_/Length_ )*diff_ *diff_ ;
		double residual_1 = ( ( m_ * (stateCat[1] - Length_) + min_value_residual) * f_);
		residual[0] = wf_1 * ( residual_0  + residual_1);

		// std::cout << std::fixed;
		// std::cout << std::setprecision(4);
		// std::cout << "CatenaryFunctor["<< stateCat[0] <<"]: residual[0]= " <<residual[0] ;
		// std::cout << " R[0]= " <<residual_0 << " R[1]= " <<residual_1; 
		// std::cout <<" , [stateCat[1]=" << stateCat[1]<< "/L_s=" << Length_ << "/d=" << dist << "] , pts_cat_coll=["
				//   <<first_coll<<"-"<<last_coll<<"/"<<last_coll-first_coll << "]" << " cat.size= " << points_catenary.size() << " is_LoS= " <<is_LoS; 
		//std::cout << std::endl ;

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
				ofs2 << residual[0] << "; "
					<< "[" << stateUGV[0] << "]; "
					<< "L= " << stateCat[1] << "; "
					<< "/L_s= " << Length_ << "; " 
					<< "/d= " << dist << "; " 
					<< "cat_coll[" << first_coll << "-" << last_coll << "/" << last_coll-first_coll << "]; "
					<< "cost_cat_=[" << cost_cat_ << "] ; "
					<< "N=[" << cost_cat_/Length_ << "] ; "
					<< "R[0]=" << residual_0 << "; R[1]=" <<residual_1 << " ; " 
				
					<< "cat.size()= " << points_catenary.size() << "; "
					<< "d_below_z_trav= " << d_below_z_trav << "; "
					<< " is_LoS= " << is_LoS <<"/" 
					<< std::endl;
			ofs2.close();
		}
	
		// std::string yy_ ;
		// yy_ = "s";
		// /********************* To obligate stop method and check Optimization result *********************/
		// 	std::cout << " Press key 'y': ";
		// 	while (yy_ != "y"){
		// 		std::cin >> yy_ ;
		// 	}
		// /**********************************************************************************************/

	return true;
    }

 	bool w_d_;
    double wf_1, wf_2, wf_3, sb_, ltm_, pr_ugv_[3];
	float m_L_c_;
	int size_;
	geometry_msgs::Vector3 pos_reel_ugv_, fp_ref_;
    pcl::KdTreeFLANN <pcl::PointXYZ> kdT_, kdT_trav_;
    pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_ , pc_trav_;
	Grid3d* g_3D_;
	ros::Publisher catenary_marker_pub_ , plane_pub_, obs_plane_pub_;
	octomap::OcTree* o_full_;
	ros::NodeHandlePtr nhP;
};

#endif