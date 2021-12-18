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
  CatenaryFunctor(double weight_factor_1, double weight_factor_2, double weight_factor_3, double safty_bound, float min_length_cat, double length_tether_max_,
  				pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points, Grid3d* grid_3D_,
                pcl::KdTreeFLANN <pcl::PointXYZ> trav_kdT, pcl::PointCloud <pcl::PointXYZ>::Ptr trav_pc, geometry_msgs::Vector3 pos_reel_ugv, int size, 
				Eigen::Vector3d fix_pos_ref, ros::NodeHandlePtr nhP, octomap::OcTree* octotree_full_, std::vector<double> &vec_l_min, bool write_data)
                : wf_1(weight_factor_1), wf_2(weight_factor_2), wf_3(weight_factor_3),sb_(safty_bound), m_L_c_(min_length_cat), ltm_(length_tether_max_), kdT_(kdT_From_NN), 
				o_p_(obstacles_Points), g_3D_(grid_3D_),kdT_trav_(trav_kdT), pc_trav_(trav_pc), pos_reel_ugv_(pos_reel_ugv),size_(size), 
				fp_ref_(fix_pos_ref), o_full_(octotree_full_), w_d_(write_data)
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
		// For residual[1] Length Cat
		double max_value_residual = 40.0;
		double min_value_residual = 10.0;

		double pos_init_cat_[3], const_collision_;
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
		// if (dist < 4.0)
		// 	safety_length = 1.010 * dist;
		// else
		// 	safety_length = 1.005 * dist;
		
		double Length_; // this value is use to ensure correct evaluation of catenary model
		if (stateCat[1] < safety_length)
			Length_ = safety_length;
		else
			Length_ = stateCat[1];

		points_catenary.clear();  dist_obst_cat.clear(); pos_cat_in_coll.clear(); cat_between_obs.clear(); 
		bc.readDataForCollisionAnalisys(g_3D_, sb_, o_full_, kdT_trav_, pc_trav_);
		bool just_one_axe = bc.configBisection(Length_, pos_init_cat_[0], pos_init_cat_[1], pos_init_cat_[2], statePosUAV[1], statePosUAV[2], statePosUAV[3], true);
		bc.getPointCatenary3D(points_catenary);
		bc.getStatusCollisionCat(dist_obst_cat, pos_cat_in_coll, cat_between_obs, first_coll, last_coll);
		bc.getMinPointZCat(min_point_z_cat, pos_in_cat_z_min);

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
			// nn.nearestObstacleStateCeres(kdT_ , x_, y_, z_, o_p_, n_coll_cat.x, n_coll_cat.y, n_coll_cat.z);
			// d_obs_= sqrt((n_coll_cat.x - x_)*(n_coll_cat.x - x_)+(n_coll_cat.y - y_)*(n_coll_cat.y - y_)+(n_coll_cat.z - z_)*(n_coll_cat.z - z_));

			// if(d_obs_ < sb_ || (i >= first_coll && i <= last_coll && i > 0) ){
			if((i >= first_coll && i <= last_coll && i > 0) ){
				if (d_obs_ < min_val_proximity_)
					d_obs_ = min_val_proximity_ ;
				if (d_obs_ < min_dist_cat_obst)
					min_dist_cat_obst = d_obs_;
				cost_ = (1.0/min_dist_cat_obst)*1.0;
				collision_ = true;
			}
			else if (d_obs_ < sb_) {
				if (d_obs_ < min_val_proximity_)
					d_obs_ = min_val_proximity_ ;
				cost_ = (1.0/d_obs_)*1.0;
			}
			else{
				if (d_obs_ < min_val_proximity_)
					d_obs_ = min_val_proximity_ ;
				cost_ = (1.0/d_obs_)*0.5;
			}
			cost_cat_ = cost_cat_ + cost_;
			// std::cout<< "		cost_cat_: "<<cost_cat_<<" , cost_: "<< cost_ << " , p=["<< x_ <<"/"<< y_ <<"/"<< z_ <<"] d_obs_= "<< d_obs_ << 
			//             " , min_val_proximity_= " << min_val_proximity_<< " , id_marker_: "<< id_marker_<<" , ["<<i<<"/"<<points_catenary.size() <<
			// 			"] , collision= "<<collision_<<std::endl;

			// Next lines compute distance between catenary lower point to traversabolity map.
			double below_z_trav[3];
			double dist_trav = 0.0;
			// nn.nearestObstacleStateCeres(kdT_trav_ , x_, y_, z_, pc_trav_, n_coll_cat.x, n_coll_cat.y, n_coll_cat.z);
			nn.nearestObstacleStateCeres(kdT_trav_ , x_, y_, z_, pc_trav_, below_z_trav[0], below_z_trav[1], below_z_trav[2]);
			dist_trav = sqrt((x_-below_z_trav[0])*(x_-below_z_trav[0])+(y_-below_z_trav[1])*(y_-below_z_trav[1])+(z_-below_z_trav[2])*(z_-below_z_trav[2]));
			if (dist_trav < min_dis_coll_cat){  
				point_coll_trav.x = below_z_trav[0];
				point_coll_trav.y = below_z_trav[1];
				point_coll_trav.z = below_z_trav[2];
				min_dis_coll_cat = dist_trav;
				if (min_point_z_cat.z < point_coll_trav.z)
					d_below_z_trav = sqrt((min_point_z_cat.z-point_coll_trav.z)*(min_point_z_cat.z-point_coll_trav.z));
			}
		}
		if (collision_)
			const_collision_ = 0.0;
		else
			const_collision_ = 0.0;
		


		/********** I Constraint to make cable far away from obstacles **********/
		
		residual[0] = wf_1 * (cost_cat_ + const_collision_);

		/********** II Constraint to make length Cable longer than euclidean distance between UAV and UGV **********/ 

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

		double m3;
		if (0.0 < d_below_z_trav)
			m3 = (max_value_residual - min_value_residual)/(sb_ - 0.0);
		else
			m3 = 0.0;
		residual[2] = wf_3 * m3 * stateCat[1] * (d_below_z_trav);

		// std::cout << std::fixed;
		// std::cout << std::setprecision(4);
		// std::cout << "CatenaryFunctor: node[" << statePosUAV[0] << "] , residual[0]= " <<residual[0] << "  residual[1]= " <<residual[1] << "  residual[2]= " <<residual[2]; 
		// std::cout <<"] , [L=" << stateCat[1]<< "/d=" << safety_length <<" , points_cat_in_coll=["
		// 		  <<first_coll<<"-"<<last_coll<<"/"<<last_coll-first_coll << "]" << " , d_below_z_trav= " << d_below_z_trav <<std::endl;

		if(w_d_){
			std::ofstream ofs;
			std::string name_output_file = "/home/simon/residuals_optimization_data/catenary.txt";
			ofs.open(name_output_file.c_str(), std::ofstream::app);
			if (ofs.is_open()) 
				ofs << residual[0] << " ; "
					<< residual[1] << " ; "
					<< residual[2] << " / "
					// << "[" << statePosUGV[0] << "] ; "
					// << "L= " << stateCat[1] << " ; "
					// << "safty_d= " << safety_length << " ; "
					// << "first_coll= " << first_coll << " ; "
					// << "last_coll= " << last_coll << " ; "
					// << "n_cat_coll= " << last_coll-first_coll << " ; "
					// << "cat.size()= " << points_catenary.size() << " ; "
					// << "d_below_z_trav= " << d_below_z_trav << " /"
					<<std::endl;
			ofs.close();
			std::ofstream ofs2;
			std::string name_output_file2 = "/home/simon/residuals_optimization_data/catenary2.txt";
			ofs2.open(name_output_file2.c_str(), std::ofstream::app);
			if (ofs2.is_open()) 
				ofs2 << residual[0] << " ; "
					<< residual[1] << " ; "
					<< residual[2] << " ; "
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
	Eigen::Vector3d fp_ref_;
	ros::Publisher catenary_marker_pub_ , plane_pub_, obs_plane_pub_;
	octomap::OcTree* o_full_;
};

#endif