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
	CatenaryLengthFunctor(){}

	struct CatenaryLength 
	{
		CatenaryLength(double weight_factor, float min_length_cat, geometry_msgs::Vector3 pos_reel_ugv, bool write_data)
					: wf_(weight_factor), m_L_c_(min_length_cat), pos_reel_ugv_(pos_reel_ugv), w_d_(write_data) 
		{}

		template <typename T>
		bool operator()(const T* const stateUGV, const T* const stateUAV, const T* const stateCat, T* residual) const 
		{
			T m_, f_;

			T ugv_reel[4];
			T pr_ugv_ [3];
			pr_ugv_[0] = T{pos_reel_ugv_.x};
			pr_ugv_[1] = T{pos_reel_ugv_.y};
			pr_ugv_[2] = T{pos_reel_ugv_.z};

			ugv_reel[0] = stateUGV[0]; 
			ugv_reel[1] = stateUGV[1]; 
			ugv_reel[2] = stateUGV[2];
			ugv_reel[3] = stateUGV[3] + pr_ugv_[2];
			T safety_length = T{m_L_c_};
			T max_value_residual = T{100.0};
			T min_value_residual = T{40.0};

			T dist = 1.005 * sqrt(pow(stateUAV[1]-ugv_reel[1],2)+pow(stateUAV[2]-ugv_reel[2],2)+pow(stateUAV[3]-ugv_reel[3],2)); 

			T diff_;
			if (stateCat[1] < dist )
				diff_ = dist - stateCat[1];
			else
				diff_ = T{0.0};

			residual[0] = wf_ *  10.0 * (exp(diff_*4.0)-1.0) ;

			// std::cout << "CatenaryLengthFunctor["<< stateCat[0]  <<"] : residual[0]= " << residual[0] << " , stateCat[1]= " << stateCat[1] << " , Length= " << Length_[1]
				 	//   << " , dist= " << dist <<	" , safety_length= " << safety_length << " ugv_reel["<<ugv_reel[1]
					//   << "," <<ugv_reel[2] <<"," <<ugv_reel[3] <<"]" << " stateUAV["<< stateUAV[1] <<","<<stateUAV[2]<<","<<stateUAV[3]<<"]" << " LoS=" <<kind_length_initial[0] <<std::endl;
			
			if(w_d_){
				std::ofstream ofs, ofs2;
				std::string name_output_file = "/home/simon/residuals_optimization_data/catenary_length.txt";
				std::string name_output_file2 = "/home/simon/residuals_optimization_data/catenary_length2.txt";
				ofs.open(name_output_file.c_str(), std::ofstream::app);
				ofs2.open(name_output_file2.c_str(), std::ofstream::app);
				if (ofs.is_open()) 
					ofs << residual[0] << "/" <<std::endl;
				ofs.close();
				if (ofs2.is_open()) 
					ofs2 << residual[0] << " ; "
					<< "(" << stateUGV[0] << "); "
					<< "stateCat= " << stateCat[1] << "; "
					<< "dist = " << dist << "; " 
					<< "diff = " << diff_  << "/" 
					<< std::endl;
				ofs2.close();
			}
			return true;
		}
		bool w_d_;
		double wf_;
		float m_L_c_;
		geometry_msgs::Vector3 pos_reel_ugv_;
	};

private:
};

#endif