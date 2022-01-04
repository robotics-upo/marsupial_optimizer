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

	struct RayCasting 
	{
		RayCasting (octomap::OcTree* o_tree_): o_t_(o_tree_)
		{
		}
		bool operator()(const double *state1, const double *state2, double *end_) const 
		{
			octomap::point3d r_;
			octomap::point3d s_(state1[1] , state1[2] , state1[3] ); //direction for rayCast
			octomap::point3d d_(state2[1]-state1[1] , state2[2]-state1[2] , state2[3]-state1[3] ); //direction for rayCast
			bool r_cast_coll = o_t_->castRay(s_, d_, r_);
			double dist_ = sqrt(pow(state2[1]-s_.x(),2)+pow(state2[2]-s_.y(),2)+pow(state2[3]-s_.z(),2));
			double distObs_ = sqrt ((s_.x()-r_.x())*(s_.x()-r_.x())+(s_.y()-r_.y())*(s_.y()-r_.y())+(s_.z()-r_.z())*(s_.z()-r_.z()) );
			if (r_cast_coll && distObs_ <= dist_)
				end_[0] = 1.0; // in case rayCast with collision
			else
				end_[0] = -1.0;  // in case rayCast without collision
			return true;
		}
		octomap::OcTree *o_t_;
	};

	struct CatenaryLength 
	{
		CatenaryLength(double weight_factor, float min_length_cat, geometry_msgs::Vector3 pos_reel_ugv, octomap::OcTree* octotree_full_, bool write_data)
					: wf_(weight_factor), m_L_c_(min_length_cat), pos_reel_ugv_(pos_reel_ugv), o_full_(octotree_full_), w_d_(write_data) 
			{
			ray_casting.reset(new ceres::CostFunctionToFunctor<1,4,4>(
							  new ceres::NumericDiffCostFunction<RayCasting, ceres::CENTRAL,1,4,4>(new RayCasting(o_full_))));
			}

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

			// std::cout << "CatenaryLengthFunctor : stateUGV["<< stateUGV[0] << "," << stateUGV[1] << "," << stateUGV[2] << "," << stateUGV[3] << 
			// 		 "] stateUAV[" << stateUAV[0] << ","<< stateUAV[1] << "," << stateUAV[2] << "," << stateUAV[3] << 
			// 		 "] ugv_reel[" << ugv_reel[0] << "," << ugv_reel[1] << "," << ugv_reel[2] << "," << ugv_reel[3] << "]"<< std::endl;

			T dist = 1.005 * sqrt(pow(stateUAV[1]-ugv_reel[1],2)+pow(stateUAV[2]-ugv_reel[2],2)+pow(stateUAV[3]-ugv_reel[3],2)); 

			// T Length_[2];
			// T kind_length_initial[1];
			// if ((ugv_reel[1]-stateUAV[1]) != 0.0 || (ugv_reel[2]-stateUAV[2]) != 0.0 || (ugv_reel[3]-stateUAV[3]) != 0.0)
			// 	(*ray_casting)(ugv_reel, stateUAV, kind_length_initial);

			// Length_[0] = stateCat[0];
			// if (kind_length_initial[0]==1.0 && safety_length > dist){
			// 	if (stateCat[1] < safety_length)
			// 		Length_[1] = safety_length;
			// 	else
			// 		Length_[1] = stateCat[1];
			// }
			// else{
				// if (stateCat[1] < dist && dist < 4.0)
				// 	Length_[1] = 1.010 * dist;
				// else if (stateCat[1] < dist && dist > 4.0)
				// 	Length_[1] = 1.005 * dist;
				// else
				// 	Length_[1] = stateCat[1];
			// }

			// if (stateCat[1] < Length_[1]) {
			// 	m_ = T{(max_value_residual - min_value_residual)/(Length_[1]*0.9 - Length_[1])};
			// 	f_ = T{1.0};
			// }
			// else{
			// 	m_ = T{0.0};
			// 	f_ = T{0.0};		
			// }
			
			// residual[0] = wf_ *  ( m_ * (stateCat[1] - Length_[1]) + min_value_residual) * f_;

			T diff_;
			if (stateCat[1] < dist )
				diff_ = dist - stateCat[1];
			else
				diff_ = T{0.0};

			residual[0] = wf_ *  (exp(diff_*2.0)-1.0) ;

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
					ofs2 << residual[0] 
					<< "[" << stateUGV[0] << "]; "
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
		octomap::OcTree* o_full_;

        std::unique_ptr<ceres::CostFunctionToFunctor<1,4,4> > ray_casting;
	};

private:
};


#endif