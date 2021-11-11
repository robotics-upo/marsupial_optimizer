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
#include "misc/catenary_solver_ceres.hpp"

#include <octomap_msgs/Octomap.h> 
#include <octomap/OcTree.h>

// #include <iostream>
// #include <fstream>
// #include <string>

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
	    NearNeighbor nn;
		CatenarySolver cS;
		bisectionCatenary bc;
		
		visualization_msgs::MarkerArray catenary_marker_, plane_marker_, obs_plane_marker_;
	    std::vector<geometry_msgs::Point> points_catenary;
		std::vector<double> dist_obst_cat; 
		std::vector<int> cat_between_obs;
		std::vector<int> pos_cat_in_coll;
        
		double d_obs_;
        double d_min = 100.0; //Initial nearest distance catenary to obstacle
		
   		int id_marker_ = statePosUAV[0];		// Related with Marker frame.id
        int n_points_cat_discard = 0;

		double m1, m2, m3;
		double max_value_residual = 200.0;
		double min_value_residual = 0.0;
		int first_coll, last_coll;

		double pos_init_cat_[3];
		pos_init_cat_[0] = statePosUGV[1]; 
		pos_init_cat_[1] = statePosUGV[2];
		pos_init_cat_[2] = statePosUGV[3] + pr_ugv_[2];

		double dist = sqrt((statePosUAV[1]-pos_init_cat_[0])*(statePosUAV[1]-pos_init_cat_[0]) + 
							(statePosUAV[2]-pos_init_cat_[1])*(statePosUAV[2]-pos_init_cat_[1]) + 
							(statePosUAV[3]-pos_init_cat_[2])*(statePosUAV[3]-pos_init_cat_[2])); 
		double safety_length ;
		if (dist < 4.0)
			safety_length = 1.010 * dist;
		else
			safety_length = 1.005 * dist;

		points_catenary.clear(); cat_between_obs.clear();  dist_obst_cat.clear(); pos_cat_in_coll.clear();

		geometry_msgs::Point min_point_z_cat, mid_point_cat, min_point_z_cat1, min_point_z_cat2;
		int pos_in_cat_z_min , mid_p_cat;
		
		bc.readDataForCollisionAnalisys(g_3D_, sb_, o_full_, kdT_trav_, pc_trav_);
		bool just_one_axe = bc.configBisection(stateCat[1], pos_init_cat_[0], pos_init_cat_[1], pos_init_cat_[2], statePosUAV[1], statePosUAV[2], statePosUAV[3], true);
		bc.getPointCatenary3D(points_catenary);
		bc.getMinPointZCat(min_point_z_cat, pos_in_cat_z_min);
		bc.getMidPointCat(mid_point_cat, mid_p_cat);
		bc.getStatusCollisionCat(dist_obst_cat, pos_cat_in_coll, cat_between_obs, first_coll, last_coll);

		mP_.markerPoints(catenary_marker_, points_catenary, id_marker_, size_, catenary_marker_pub_);

		int num_point_per_segment = 1;
		int num_segment = ceil(( points_catenary.size() )/num_point_per_segment); // previously (n_points_cat_discard+n_points_cat_discard/2.0)
		if (num_segment <= 0)
			num_segment = 1;

		// Next lines compute distance between catenary lower point to traversabolity map.
		double below_z_trav[3];
		double d_below_z_trav = 0.0;
		nn.nearestObstacleStateCeres(kdT_trav_ , min_point_z_cat.x, min_point_z_cat.y, min_point_z_cat.z, pc_trav_, below_z_trav[0], below_z_trav[1], below_z_trav[2]);
		if (min_point_z_cat.z <= below_z_trav[2] + sb_/2.0){ 
			d_below_z_trav = sqrt((min_point_z_cat.x-below_z_trav[0])*(min_point_z_cat.x-below_z_trav[0])+
								(min_point_z_cat.y-below_z_trav[1])*(min_point_z_cat.y-below_z_trav[1])+
								(min_point_z_cat.z-below_z_trav[2])*(min_point_z_cat.z-below_z_trav[2]));
		}
		// Next lines defined bound min and max to not get collision in catenary 
		double L_, L_max, L_min, L_new, length_less_coll_;
		double not_catenary_ = 0.0;
		if (!just_one_axe){
			L_max = L_min = L_ = L_new = safety_length;
			int count_ = 0;
			bool bound_finded_ = false;
			bool starting_in_coll = false;
			length_less_coll_ = safety_length;
			int min_num_coll_ = 10000;
			std::vector<double> dist_obst_cat_; 
			std::vector<int> pos_cat_in_coll_, cat_between_obs_;
			dist_obst_cat_.clear() ; pos_cat_in_coll_.clear(); cat_between_obs_.clear();
			geometry_msgs::Point p_min;
			double t_min[3];
			int n_min, f_coll, l_coll;
			while (L_ <= ltm_ || !bound_finded_){
				if (safety_length > 4.0)
						L_ = safety_length * (1.000 + 0.001*count_);
					else
						L_ = safety_length * (1.000 + 0.002*count_);
				if( ltm_ < L_){
					if(!starting_in_coll){
						L_max = L_new; 
						break;
					}
					else{
						L_max = L_min = length_less_coll_;
						not_catenary_ = 1.0;
						// ROS_ERROR("NOT SOLUTION FOR CATENARY 1: Always in collision point [%i]",id_marker_);
						break;
					}
				}
				bc.configBisection(L_, pos_init_cat_[0], pos_init_cat_[1], pos_init_cat_[2], statePosUAV[1], statePosUAV[2], statePosUAV[3],true);
				bc.getPointCatenary3D(points_catenary);
				bc.getMinPointZCat(p_min, n_min);
				nn.nearestObstacleStateCeres(kdT_trav_ , p_min.x, p_min.y, p_min.z, pc_trav_, t_min[0], t_min[1], t_min[2]);
				bc.getStatusCollisionCat(dist_obst_cat_, pos_cat_in_coll_, cat_between_obs_, f_coll, l_coll);
				if (p_min.z < t_min[2] + 0.05){ //If catenary is not feasible for UGV and UAV position
					if (starting_in_coll){
						not_catenary_ = 1.0;
						L_max = L_min = length_less_coll_;
						// ROS_ERROR("NOT SOLUTION FOR CATENARY 2: Always in collision point [%i] length_less_coll_=%f",id_marker_,length_less_coll_);
						break;
						}
					L_max = L_new;
					break;
				}
				if (pos_cat_in_coll_.size() > 0){
					if (count_== 0)
						starting_in_coll = true;
					if (!starting_in_coll){
						L_max = L_new; // Is the previous L value
						bound_finded_ = true;
					}
				}
				if (pos_cat_in_coll_.size()==0 && starting_in_coll){
					L_min = L_;  // Is the current L value 
					bound_finded_ = true;
				}
				if(min_num_coll_ > pos_cat_in_coll_.size()){
					length_less_coll_ = L_;  // Value to use in case of not solution because is the best option
					min_num_coll_ = pos_cat_in_coll_.size();
				}
				if (bound_finded_)
					break;
				L_new = L_;
				count_++;
			}
			if(starting_in_coll && not_catenary_==0.0){
				L_ = L_min;
				L_new = L_min;
				bound_finded_ = false;
				count_ = 0;
				while (L_ <= ltm_ || !bound_finded_){
					if (L_min > 4.0)
						L_ = L_min * (1.001 + 0.001*count_);
					else
						L_ = L_min * (1.001 + 0.004*count_);
					if( (ltm_ - L_) < 0.0){
						L_max = L_new; 
						break;
					}
					bc.configBisection(L_, pos_init_cat_[0], pos_init_cat_[1], pos_init_cat_[2], statePosUAV[1], statePosUAV[2], statePosUAV[3],true);
					bc.getPointCatenary3D(points_catenary);
					bc.getMinPointZCat(p_min, n_min);
					nn.nearestObstacleStateCeres(kdT_trav_ , p_min.x, p_min.y, p_min.z, pc_trav_, t_min[0], t_min[1], t_min[2]);
					bc.getStatusCollisionCat(dist_obst_cat_, pos_cat_in_coll_, cat_between_obs_, f_coll, l_coll);
					if (p_min.z < t_min[2] ){ 
						L_max = L_new;
						break;
					}
					if (pos_cat_in_coll_.size() > 0){
						L_max = L_new; 
						bound_finded_ = true;
						break;
					}
					if (bound_finded_)
						break;
					L_new = L_;
					count_++;
				}
			}
		}
		else{
			ROS_WARN("SOLUTION FOR CATENARY in one axes");
			L_max = L_min = safety_length;
		}	
		
		/********** I Constraint to make cable far away from obstacles **********/
		
		double status_length_, num_point_coll;
		num_point_coll = 0.0;

		//First identify how many catenary points are in collision
		if(last_coll != 0  && first_coll != 0 && last_coll == first_coll)
			num_point_coll = 1.0;
		else
			num_point_coll = last_coll- first_coll;

		//Second Identify type of collision, close to the Lmin or Lmax
		if (not_catenary_ == 1.0)
			status_length_ = 0.0;
		else if (num_point_coll > 0.0 && stateCat[1] <= (L_max-L_min)/2.0 ) // second condition when catenary is inside safety area but a point is close to collision
			status_length_ = 1.0/stateCat[1];
		else if (num_point_coll > 0.0 && stateCat[1] > (L_max-L_min)/2.0 ) // second condition when catenary is inside safety area but a point is close to collision
			status_length_ = stateCat[1];


		residual[0] = wf_1 * (status_length_ * 10.0 * (sb_ * num_point_coll)+ not_catenary_ * 200.0 * (sb_ * num_segment) );

		/********** II Constraint to make length Cable longer than distance between UAV and UGV **********/ 

		double min_value_residual1, cost1_, cost2_, cost3_;
		cost1_ = cost2_ = cost3_ = 0.0;
		if (not_catenary_ == 1.0)
			cost3_ = 1.0;
		else if (stateCat[1] < L_min) {
			min_value_residual1 = 10.0;
			m2 = (max_value_residual - min_value_residual1)/(L_min*0.9 - L_min);
			cost1_ = 1.0;
		}
		else if (stateCat[1] > L_max){
			min_value_residual1 = 10.0;
			m2 = (max_value_residual - min_value_residual1)/(L_max*1.1 - L_max);
			cost2_ = 1.0;
		}
		
		residual[1] = wf_2 * ( cost1_* (1.0/stateCat[1]) *( m2 * (stateCat[1] - L_min) + min_value_residual1) + 
							   cost2_* ( stateCat[1] ) * ( m2 * (stateCat[1] - L_max) + min_value_residual1) +
							   cost3_* 2000.0);

		/********** III Constraint to make cable not place below traversable map **********/

		if (0.0 < d_below_z_trav)
			m3 = (max_value_residual - min_value_residual)/(sb_ - 0.0);
		else
			m3 = 0.0;
		residual[2] = wf_3 * m3 * stateCat[1] * (d_below_z_trav);

		double min_dist = 1000.0;
		int pos_min_dist_= 0;
		for (int i=0 ; i < dist_obst_cat.size() ; i++){
			if (min_dist > dist_obst_cat[i]){
				min_dist = dist_obst_cat[i];
				pos_min_dist_ = i;
			}
		}

		std::cout << std::fixed;
		std::cout << std::setprecision(5);
		std::cout << "CatenaryFunctor: node[" << statePosUAV[0] << "/" << statePosUGV[0] << "]  , residual[0]= " <<residual[0] << " residual[1]= " <<residual[1] << " residual[2]= " <<residual[2]; 
		std::cout << " , 	there_is_not_cat=[" << not_catenary_ <<"] , [L=" << stateCat[1]<< "/Lmin=" << L_min << "/Lmax=" << L_max << "/d=" << safety_length <<" , points_cat_in_coll=["
				  <<first_coll<<"-"<<last_coll<<"/"<<last_coll-first_coll<<"/"<<num_segment<<"]"<<std::endl;

		// std::ofstream ofs;
		// std::string name_output_file = "/home/simon/residuals_optimization_data/catenary.txt";
		// ofs.open(name_output_file.c_str(), std::ofstream::app);
		// if (ofs.is_open()) 
		// 	ofs << residual[0] << " ; "
		// 		<< residual[1] << " ; "
		// 		<< residual[2] << " ; "
		// 		<<std::endl;
		// ofs.close();
	
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