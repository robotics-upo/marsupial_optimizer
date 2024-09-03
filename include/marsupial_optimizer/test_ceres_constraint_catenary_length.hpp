#ifndef TEST_CERES_CONSTRAINS_CATENARY_LENGTH_HPP
#define TEST_CERES_CONSTRAINS_CATENARY_LENGTH_HPP

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
#include "catenary_checker/bisection_catenary_3D.h"

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

class TestCatenaryLengthFunctor {

public:
	TestCatenaryLengthFunctor(double w_f, float min_l_cat, geometry_msgs::Point v_initial, geometry_msgs::Point v_final, bool w_data)
                : wf_(w_f), m_L_c_(min_l_cat), v_i_(v_initial), v_f_(v_final), w_d_(w_data) {}

  	template <typename T>
  	bool operator()(const T* const stateCat, T* residual) const 
{
  	T m_, f_;

	T safety_length = T{m_L_c_};
	T max_value_residual = T{100.0};
	T min_value_residual = T{10.0};

	double _d = sqrt((v_i_.x-v_f_.x)*(v_i_.x-v_f_.x)+(v_i_.y-v_f_.y)*(v_i_.y-v_f_.y)+(v_i_.z-v_f_.z)*(v_i_.z-v_f_.z)); 
	T dist = T{_d};

	T Length_; // this value is use to ensure correct evaluation of catenary model
	if (dist < safety_length){
		if (stateCat[0] < safety_length)
			Length_ = safety_length;
		else
			Length_ = stateCat[0];
	}
	else{
		if (stateCat[0] < dist && dist < 4.0)
			Length_ = T{1.010 * dist};
		else if (stateCat[0] < dist && dist > 4.0)
			Length_ = T{1.005 * dist};
		else
			Length_ = stateCat[0];
	}

	if (stateCat[0] < Length_) {
		m_ = (max_value_residual - min_value_residual)/(Length_*0.9 - Length_);
		f_ = T{1.0};
	}
	else{
		m_ = T{0.0};
		f_ = T{0.0};		
	}
	
	residual[0] = wf_ *  ( m_ * (stateCat[0] - Length_) + min_value_residual) * f_;

	// std::cout << "TestCatenaryLengthFunctor : residual[0]= " << residual[0] << " , stateCat[0]= " << stateCat[0] << " , Length_= " << Length_
	// 	 	  << " , dist= " << dist <<	" , safety_length= " << safety_length <<std::endl;
	
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
geometry_msgs::Point v_i_, v_f_;

private:
};

#endif