#ifndef TEST_CERES_CONSTRAINS_CATENARY_HPP
#define TEST_CERES_CONSTRAINS_CATENARY_HPP

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

#include "catenary_checker/bisection_catenary_3D.h"

#include <octomap_msgs/Octomap.h> 
#include <octomap/OcTree.h>

#include <iostream>
#include <fstream>
#include <string>

#define PRINTF_REGULAR "\x1B[0m"
#define PRINTF_RED "\x1B[31m"
#define PRINTF_GREEN "\x1B[32m"
#define PRINTF_YELLOW "\x1B[33m"
#define PRINTF_BLUE "\x1B[34m"
#define PRINTF_MAGENTA "\x1B[35m"
#define PRINTF_CYAN "\x1B[36m"
#define PRINTF_WHITE "\x1B[37m"
#define PRINTF_ORANGE  "\x1B[38;2;255;128;0m"
#define PRINTF_ROSE    "\x1B[38;2;255;151;203m"
#define PRINTF_LBLUE   "\x1B[38;2;53;149;240m"
#define PRINTF_LGREEN  "\x1B[38;2;17;245;120m"
#define PRINTF_GRAY    "\x1B[38;2;176;174;174m"

using ceres::AutoDiffCostFunction;
using ceres::NumericDiffCostFunction;
using ceres::CENTRAL;
using ceres::FORWARD;
using ceres::RIDDERS;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;


struct TestCatenaryFunctor {
  TestCatenaryFunctor(double w_f, double safty_bound, float min_l_cat, bool w_data, geometry_msgs::Point v_initial, geometry_msgs::Point v_final,
				ros::NodeHandlePtr nhP_, octomap::OcTree* octotree_full_, 
  				pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points, Grid3d* grid_3D_,
                pcl::KdTreeFLANN <pcl::PointXYZ> trav_kdT, pcl::PointCloud <pcl::PointXYZ>::Ptr trav_pc)
                : wf_(w_f), sb_(safty_bound), m_L_c_(min_l_cat), w_d_(w_data), v_i_(v_initial), v_f_(v_final), 
				o_full_(octotree_full_), kdT_(kdT_From_NN), 
				o_p_(obstacles_Points), g_3D_(grid_3D_),kdT_trav_(trav_kdT), pc_trav_(trav_pc)
    {
		nhP = nhP_;
		catenary_marker_pub_ = nhP_->advertise<visualization_msgs::MarkerArray>("catenary_marker", 1);
		plane_pub_ = nhP_->advertise<visualization_msgs::MarkerArray>("plane_markerArray", 1);
		obs_plane_pub_ = nhP_->advertise<visualization_msgs::MarkerArray>("obsplane_markerArray", 1);
    }

    bool operator()(const double* stateCat, double* residual) const 
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
		double max_value_residual = 10000.0;
		double min_value_residual = 1000.0;
		// For Marker
   		int id_marker_ = 1;		// Related with Marker frame.id

		double dist = sqrt((v_f_.x-v_i_.x)*(v_f_.x-v_i_.x) + (v_f_.y-v_i_.y)*(v_f_.y-v_i_.y) + (v_f_.z-v_i_.z)*(v_f_.z-v_i_.z)); 

		double cost_cat_ = 0.0;
		double d_obs_;
		double min_val_proximity_ = 0.015;
		double min_dist_cat_obst = 1000.0;
		double safety_length = m_L_c_;
		
		double Length_, const_collision_; // this value is use to ensure correct evaluation of catenary model
		if (dist < safety_length){
			if (stateCat[0] < safety_length)
				Length_ = safety_length;
			else
				Length_ = stateCat[0];
		}
		else{
			if (stateCat[0] < dist && dist < 4.0)
				Length_ = 1.010 * dist;
			else if (stateCat[0] < dist && dist > 4.0)
				Length_ = 1.005 * dist;
			else
				Length_ = stateCat[0];
		}

		// std::cout << "TestCatenaryFunctor: node[" << statePosUAV[0] << "]" << "[" << statePosUAV[1] << "," << statePosUAV[2] << "," <<statePosUAV[3]<< "]" <<std::endl;
		points_catenary.clear();  dist_obst_cat.clear(); pos_cat_in_coll.clear(); cat_between_obs.clear(); 
		bc.readDataForCollisionAnalisys(g_3D_, sb_, o_full_, kdT_trav_, pc_trav_);
		bool just_one_axe = bc.configBisection(Length_, v_i_.x, v_i_.y, v_i_.z, v_f_.x, v_f_.y, v_f_.z, true);
		bc.getPointCatenary3D(points_catenary);
		bc.getStatusCollisionCat(dist_obst_cat, pos_cat_in_coll, cat_between_obs, first_coll, last_coll);
		bc.getMinPointZCat(min_point_z_cat, pos_in_cat_z_min);

		mP_.markerPoints(catenary_marker_, points_catenary, id_marker_, 1, catenary_marker_pub_);


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
				cost_ = (1.0/min_val_proximity_)*6.0;
				collision_ = true;
			}
			else if (d_obs_ < sb_) {
				cost_ = (1.0/min_val_proximity_)*6.0;
				collision_ = true;
			}
			else{
				cost_ = (1.0/d_obs_)*1.0;
			}
			cost_cat_ = cost_cat_ + cost_;
		}

		// if (collision_)
		// 	const_collision_ = 0.0;
		// else
		// 	const_collision_ = 0.0;


		double m_, f_;
		if (stateCat[0] < Length_) {
			m_ = (max_value_residual - min_value_residual)/(Length_*0.9 - Length_);
			f_ = 1.0;
		}
		else{
			m_ = 0.0;
			f_ = 0.0;		
		}

		/********** I Constraint to make cable far away from obstacles **********/
		
		// residual[0] = wf_ * (cost_cat_ + const_collision_);

		double residual_1 = ( ( m_ * (stateCat[0] - Length_) + min_value_residual) * f_);
		double residual_0 = (cost_cat_);
		residual[0] = wf_ * ( residual_0  + residual_1);

		// residual[1] = 0.1 * ( m_ * (stateCat[0] - Length_) + min_value_residual) * f_;


		std::cout << std::fixed;
		std::cout << std::setprecision(4);
		std::cout << "TestCatenaryFunctor: residual[0]= " <<residual[0] ;
		// std::cout << " residual[1]= " <<residual[1] << " cost_cat="<< cost_cat_; 
		std::cout << " R[0]= " <<residual_0 << " R[1]= " <<residual_1; 
		std::cout <<" , [stateCat[0]=" << stateCat[0]<< "/L_s=" << Length_ << "/d=" << dist << "] , pts_cat_coll=["
				  <<first_coll<<"-"<<last_coll<<"/"<<last_coll-first_coll << "]" << " cat.size= " << points_catenary.size();

		if(w_d_){
			std::ofstream ofs;
			std::string name_output_file = "/home/simon/residuals_optimization_data/test_catenary.txt";
			ofs.open(name_output_file.c_str(), std::ofstream::app);
			if (ofs.is_open()) 
				ofs << residual[0] << " / "
					<< std::endl;
			ofs.close();
			std::ofstream ofs2;
			std::string name_output_file2 = "/home/simon/residuals_optimization_data/test_catenary2.txt";
			ofs2.open(name_output_file2.c_str(), std::ofstream::app);
			if (ofs2.is_open()) 
				ofs2 << residual[0] << " ; "
					<< "L= " << stateCat[0] << " ; "
					<< "safty_d= " << safety_length << " ; "
					<< "first_coll= " << first_coll << " ; "
					<< "last_coll= " << last_coll << " ; "
					<< "n_cat_coll= " << last_coll-first_coll << " ; "
					<< "cat.size()= " << points_catenary.size() << " ; "
					<< "d_below_z_trav= " << d_below_z_trav << " /"
					<<std::endl;
			ofs2.close();
		}
	
		std::string yy_ ;
		yy_ = "s";
		/********************* To obligate stop method and check Optimization result *********************/
			std::cout << "  Press key 'y': ";
			while (yy_ != "y"){
				std::cin >> yy_ ;
			}
		/**********************************************************************************************/

	return true;
    }

 	bool w_d_;
    double wf_, sb_;
	float m_L_c_;
	geometry_msgs::Point v_i_, v_f_;
    pcl::KdTreeFLANN <pcl::PointXYZ> kdT_;
    pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_;
	Grid3d* g_3D_;
	pcl::KdTreeFLANN <pcl::PointXYZ> kdT_trav_;
    pcl::PointCloud <pcl::PointXYZ>::Ptr pc_trav_;
	ros::Publisher catenary_marker_pub_ , plane_pub_, obs_plane_pub_;
	octomap::OcTree* o_full_;
	ros::NodeHandlePtr nhP;

};

#endif