#ifndef CERES_CONSTRAINS_CATENARY_LENGTH_HPP
#define CERES_CONSTRAINS_CATENARY_LENGTH_HPP

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

class CatenaryLengthFunctor {

public:
	CatenaryLengthFunctor(double weight_factor, float min_length_cat, geometry_msgs::Vector3 pos_reel_ugv, bool write_data)
                : wf_(weight_factor), m_L_c_(min_length_cat), pos_reel_ugv_(pos_reel_ugv), w_d_(write_data) {}

  	template <typename T>
  	bool operator()(const T* const stateUAV, const T* const stateUGV, const T* const stateCat, T* residual) const 
{
  	T m_, f_;

	T p_reel[3], const_collision_;
	p_reel[0] = stateUGV[1]; 
	p_reel[1] = stateUGV[2];
	p_reel[2] = stateUGV[3] + pos_reel_ugv_.z;
	T cost_cat_ = T{0.0};
	T d_obs_;
	T min_val_proximity_ = T{0.015};
	T min_dist_cat_obst = T{1000.0};
	T safety_length = T{m_L_c_};
	T max_value_residual = T{20.0};
	T min_value_residual = T{0.0};

	T dist = sqrt((stateUAV[1]-p_reel[0])*(stateUAV[1]-p_reel[0])+(stateUAV[2]-p_reel[1])*(stateUAV[2]-p_reel[1])+(stateUAV[3]-p_reel[2])*(stateUAV[3]-p_reel[2])); 

	T Length_; // this value is use to ensure correct evaluation of catenary model
	if (dist < safety_length){
		if (stateCat[1] < safety_length)
			Length_ = safety_length;
		else
			Length_ = stateCat[1];
	}
	else{
		if (stateCat[1] < dist && dist < 4.0)
			Length_ = T{1.010 * dist};
		else if (stateCat[1] < dist && dist > 4.0)
			Length_ = T{1.005 * dist};
		else
			Length_ = stateCat[1];
	}

	if (stateCat[1] < Length_) {
		m_ = (max_value_residual - min_value_residual)/(Length_*0.9 - Length_);
		f_ = T{1.0};
	}
	else{
		m_ = T{0.0};
		f_ = T{0.0};		
	}
	
	residual[0] = wf_ *  ( m_ * (stateCat[1] - Length_) + min_value_residual) * f_;

	// std::cout << "CatenaryLengthFunctor : residual[0]= " << residual[0] << " , stateCat[1]= " << stateCat[1] << " , Length_= " << Length_
	// 	 	  << " , dist= " << dist <<	" , safety_length= " << safety_length << " p_reel["<<p_reel[0]
	// 		  << "," <<p_reel[1] <<"," <<p_reel[2] <<"]" << " stateUAV["<< stateUAV[1] <<","<<stateUAV[2]<<","<<stateUAV[3]<<"]" <<std::endl;
	
	if(w_d_){
		std::ofstream ofs;
		std::string name_output_file = "/home/simon/residuals_optimization_data/catenary_length.txt";
		ofs.open(name_output_file.c_str(), std::ofstream::app);
		if (ofs.is_open()) 
			ofs << residual[0] << "/" <<std::endl;
		ofs.close();
	}
	return true;
}

bool w_d_;
double wf_;
float m_L_c_;
geometry_msgs::Vector3 pos_reel_ugv_;

private:
};

#endif