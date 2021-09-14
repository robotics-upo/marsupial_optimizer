#ifndef CERES_CONSTRAINS_CATENARY_HPP
#define CERES_CONSTRAINS_CATENARY_HPP


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

#include "misc/near_neighbor.hpp"
#include "misc/bisection_catenary_3D.h"
#include "misc/marker_publisher.hpp"
#include "misc/catenary_solver_ceres.hpp"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;


struct CatenaryFunctor {
  CatenaryFunctor(double weight_factor_1, double weight_factor_2, double weight_factor_3, double safty_bound, 
  				pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points, Grid3d* grid_3D_,
                pcl::KdTreeFLANN <pcl::PointXYZ> trav_kdT, pcl::PointCloud <pcl::PointXYZ>::Ptr trav_pc, geometry_msgs::Point pos_reel_ugv, int size, 
				double pos_reel_z, Eigen::Vector3d fix_pos_ref, ros::NodeHandlePtr nhP)
                : wf_1(weight_factor_1), wf_2(weight_factor_2), wf_3(weight_factor_3),sb_(safty_bound), kdT_(kdT_From_NN), o_p_(obstacles_Points), 
				g_3D_(grid_3D_),kdT_trav_(trav_kdT), pc_trav_(trav_pc), pos_reel_ugv_(pos_reel_ugv),s_(size), pr_z_(pos_reel_z), fp_ref_(fix_pos_ref)
    {
		catenary_marker_pub_ = nhP->advertise<visualization_msgs::MarkerArray>("catenary_marker", 1);
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
		
		visualization_msgs::MarkerArray catenary_marker_;
	    std::vector<geometry_msgs::Point> points_catenary;
        
		double d_obs_;
        double d_min = 100.0; //Initial nearest distance catenary to obstacle
		
   		int id_marker_ = statePosUAV[0];		// Related with Marker frame.id
        int n_points_cat_dis;

		double m1, m2, m3;
		double max_value_residual = 100.0;
		double min_value_residual = 0.0;

		double pos_init_cat_[3];
		pos_init_cat_[0] = statePosUGV[1]; 
		pos_init_cat_[1] = statePosUGV[2];
		pos_init_cat_[2] = statePosUGV[3] + pr_z_;

		double dist_ = sqrt((statePosUAV[1]-pos_init_cat_[0])*(statePosUAV[1]-pos_init_cat_[0]) + 
							(statePosUAV[2]-pos_init_cat_[1])*(statePosUAV[2]-pos_init_cat_[1]) + 
							(statePosUAV[3]-pos_init_cat_[2])*(statePosUAV[3]-pos_init_cat_[2])); 
		double safety_length = 1.01 * dist_;


		points_catenary.clear();

		geometry_msgs::Point min_point_z_cat;
		int pos_in_cat_z_min , pos_in_cat_col_trav1, pos_in_cat_col_trav2, pos_segment_col_trav1, pos_segment_col_trav2;
		bc.configBisection(stateCat[1], pos_init_cat_[0], pos_init_cat_[1], pos_init_cat_[2], statePosUAV[1], statePosUAV[2], statePosUAV[3]);
		bc.getPointCatenary3D(points_catenary);
		bc.getMinPointZCat(min_point_z_cat, pos_in_cat_z_min);

		mP_.markerPoints(catenary_marker_, points_catenary, id_marker_, s_, catenary_marker_pub_);

		n_points_cat_dis = 2;
		std::vector<geometry_msgs::Point> nearest_p_cat_to_obs;
		std::vector<double> min_dist_segment;
		std::vector<int> pos_in_cat_col_trav;
		nearest_p_cat_to_obs.clear(); min_dist_segment.clear(); pos_in_cat_col_trav.clear(); 
		int num_point_per_segment = 2;
		int num_segment = ceil(( points_catenary.size() - ( n_points_cat_dis + n_points_cat_dis/2.0 ) )/num_point_per_segment);
		if (num_segment <= 0)
			num_segment = 1;
		for (int i=0 ; i< num_segment ; i++){
			geometry_msgs::Point _p;
			_p.x = -1.0; 
			_p.y = -1.0; 
			_p.z = -1.0; 
			nearest_p_cat_to_obs.push_back(_p);
			pos_in_cat_col_trav.push_back(-1);
			min_dist_segment.push_back(0.0);
		}

		int count_num_point_segment = 0;
		int count_segment= 0 ;
		bool aux_count = 0;
		std::vector <int> obs_in_segment_close_cat;
		obs_in_segment_close_cat.clear();
		int pos_in_segment_z_min = 0;
		double able_average_segment[num_segment];
		able_average_segment[0] = 0.0;
		int aqui = 0;
		for(size_t i = 0 ; i < points_catenary.size() ; i++){
			double near_[3];
			if (i >= n_points_cat_dis && (i < points_catenary.size() - n_points_cat_dis/2.0) ){
				TrilinearParams d = g_3D_->getPointDistInterpolation((double)points_catenary[i].x, (double)points_catenary[i].y, (double)points_catenary[i].z);
				double x , y, z;
				x = points_catenary[i].x;
				y = points_catenary[i].y;
				z = points_catenary[i].z;
				d_obs_= (d.a0 + d.a1*x + d.a2*y + d.a3*z + d.a4*x*y + d.a5*x*z + d.a6*y*z + d.a7*x*y*z); 
				// Next lines to get nearest obstacle to each segment
				count_num_point_segment++;	
				if (d_obs_ < d_min){
					// nn.nearestObstacleStateCeres(kdT_ , points_catenary[i].x, points_catenary[i].y, points_catenary[i].z, o_p_, near_[0], near_[1], near_[2]);
					min_dist_segment[count_segment] = d_obs_;
					d_min = d_obs_;
					nearest_p_cat_to_obs[count_segment].x = points_catenary[i].x; 
					nearest_p_cat_to_obs[count_segment].y = points_catenary[i].y; 
					nearest_p_cat_to_obs[count_segment].z = points_catenary[i].z; 
					pos_in_cat_col_trav[count_segment] = i;
					if (aux_count && d_obs_ < sb_){
						double _d=0;
						if(count_segment > 0){
						_d = sqrt((points_catenary[i].x-nearest_p_cat_to_obs[count_segment-1].x)*(points_catenary[i].x-nearest_p_cat_to_obs[count_segment-1].x) + 
								 (points_catenary[i].y-nearest_p_cat_to_obs[count_segment-1].y)*(points_catenary[i].y-nearest_p_cat_to_obs[count_segment-1].y) +
								 (points_catenary[i].z-nearest_p_cat_to_obs[count_segment-1].z)*(points_catenary[i].z-nearest_p_cat_to_obs[count_segment-1].z));
						}
						if(_d > sb_ || count_segment == 0)
						able_average_segment[count_segment] = 1.0;
						obs_in_segment_close_cat.push_back(count_segment);
						aux_count = false;
						aqui++;
						// printf(" aqui[%i] = %i \n", aqui, count_segment);
					}	
				}
				if (i == pos_in_cat_z_min)
					pos_in_segment_z_min = count_segment;
				if (count_num_point_segment >= num_point_per_segment){
					count_num_point_segment = 0;
					count_segment++;
					aux_count = true;
					d_min = 100.0;
					able_average_segment[count_segment] = 0.0;
				}
			}
		}

		ROS_INFO ("obs_in_segment_close_cat = [%lu/%i]",obs_in_segment_close_cat.size(),num_segment);
		double below_z_trav[3];
		double d_below_z_trav = 0.0;
		nn.nearestObstacleStateCeres(kdT_trav_ , min_point_z_cat.x, min_point_z_cat.y, min_point_z_cat.z, pc_trav_, below_z_trav[0], below_z_trav[1], below_z_trav[2]);
		if (min_point_z_cat.z <= below_z_trav[2] + sb_){ 
			d_below_z_trav = sqrt((min_point_z_cat.x-below_z_trav[0])*(min_point_z_cat.x-below_z_trav[0])+
								(min_point_z_cat.y-below_z_trav[1])*(min_point_z_cat.y-below_z_trav[1])+
								(min_point_z_cat.z-below_z_trav[2])*(min_point_z_cat.z-below_z_trav[2]));
		}
			
		
		/********** I Constraint to make cable far away from obstacles **********/
		
		double cost_function_obstacles_ = 0.0;
		double dist_obs_cat_ = 0.0;
		int count_aux = 0;
		for (int i=0 ; i < num_segment ; i++){
			if(obs_in_segment_close_cat.size() == 0 && stateCat[1] > safety_length ){
					ROS_ERROR("NO hay colision en catenaria");
					break;
			}
			// else if(stateCat[1] < safety_length){
			// 	m1 = (max_value_residual - min_value_residual)/(safety_length*0.9 - safety_length);
			// 	cost_function_obstacles_ = (wf_1*1000.0) * m1 * (stateCat[1] - safety_length);
			// 	ROS_ERROR("Hay colision en catenaria por que stateCat[1] > safety_length");
			// }		
			else if (obs_in_segment_close_cat.size() == 1){
				if (obs_in_segment_close_cat[0]==i){ 
					double d_,x_cat_,y_cat_,z_cat_;
					if (min_dist_segment[i] > sb_){
						d_ = 0.0;
						m1 = 0.0;
					}
					else{
						// double L1 = stateCat[1] * 1.1;
						// double L2 = stateCat[1] * 0.9;
						// std::vector<geometry_msgs::Point> p_c;
						// std::vector<double> m_d_s;
						// int c_s= 0 ;
						// int c_n_p_s = 0;
						// double d_m = 100.0;
						// bool a_c = true;
						// bc.configBisection(L1, pos_init_cat_[0], pos_init_cat_[1], pos_init_cat_[2], statePosUAV[1], statePosUAV[2], statePosUAV[3]);
						// bc.getPointCatenary3D(p_c);
						// for(size_t i = 0 ; i < p_c.size() ; i++){
						// 	double near_[3];
						// 	if (i >= n_points_cat_dis && (i < p_c.size() - n_points_cat_dis/2.0) ){
						// 		TrilinearParams d = g_3D_->getPointDistInterpolation((double)p_c[i].x, (double)p_c[i].y, (double)p_c[i].z);
						// 		double x , y, z;
						// 		x = p_c[i].x;
						// 		y = p_c[i].y;
						// 		z = p_c[i].z;
						// 		d_obs_= (d.a0 + d.a1*x + d.a2*y + d.a3*z + d.a4*x*y + d.a5*x*z + d.a6*y*z + d.a7*x*y*z); 
						// 		// Next lines to get nearest obstacle to each segment
						// 		c_n_p_s++;	
						// 		if (d_obs_ < d_m){
						// 			m_d_s[c_s] = d_obs_;
						// 			d_m = d_obs_;
						// 			nearest_p_cat_to_obs[c_s].x = p_c[i].x; 
						// 			nearest_p_cat_to_obs[c_s].y = p_c[i].y; 
						// 			nearest_p_cat_to_obs[c_s].z = p_c[i].z; 
						// 			pos_in_cat_col_trav[c_s] = i;
						// 			if (a_c && d_obs_ < sb_){
						// 				double _d=0;
						// 				if(c_s > 0){
						// 				_d = sqrt((p_c[i].x-nearest_p_cat_to_obs[c_s-1].x)*(p_c[i].x-nearest_p_cat_to_obs[c_s-1].x) + 
						// 						(p_c[i].y-nearest_p_cat_to_obs[c_s-1].y)*(p_c[i].y-nearest_p_cat_to_obs[c_s-1].y) +
						// 						(p_c[i].z-nearest_p_cat_to_obs[c_s-1].z)*(p_c[i].z-nearest_p_cat_to_obs[c_s-1].z));
						// 				}
						// 				if(_d > sb_ || c_s == 0)
						// 				able_average_segment[c_s] = 1.0;
						// 				obs_in_segment_close_cat.push_back(c_s);
						// 				a_c = false;
						// 			}	
						// 		}
						// 		if (c_n_p_s >= num_point_per_segment){
						// 			c_n_p_s = 0;
						// 			c_s++;
						// 			a_c = true;
						// 			d_m = 100.0;
						// 			able_average_segment[c_s] = 0.0;
						// 		}
						// 	}
						// }



						// m1 = (max_value_residual - min_value_residual)/(1.0 - 0.0);
						m1 = 1000.0;
						x_cat_ = nearest_p_cat_to_obs[i].x;
						y_cat_ = nearest_p_cat_to_obs[i].y;
						z_cat_ = nearest_p_cat_to_obs[i].z;
						d_ = sqrt((statePosUAV[1]-x_cat_)*(statePosUAV[1]-x_cat_)+(statePosUAV[2]-y_cat_)*(statePosUAV[2]-y_cat_)+(statePosUAV[3]-z_cat_)*(statePosUAV[3]-z_cat_)); 
						ROS_ERROR("Es 1 :  segment=%i , cost_function_obstacles_=%f , d_=%f , m1=%f p_cat=[%f %f %f]", i, cost_function_obstacles_,d_,m1,x_cat_,y_cat_,z_cat_);
					}
					cost_function_obstacles_ = wf_1 * m1 * (d_);
				}
			}
			else if(obs_in_segment_close_cat.size()==2){
				if (obs_in_segment_close_cat[0]==i || obs_in_segment_close_cat[1]==i){
					double d_,x_cat_,y_cat_,z_cat_;
					if (min_dist_segment[i] > sb_){
						m1 = 0.0;
						dist_obs_cat_= 0.0;
					}
					else{
						// m1 = (max_value_residual - min_value_residual)/(1.0 - 0.0);
						m1 = 100.0;
						x_cat_ = nearest_p_cat_to_obs[i].x;
						y_cat_ = nearest_p_cat_to_obs[i].y;
						z_cat_ = nearest_p_cat_to_obs[i].z;
						double n_[3];
						nn.nearestObstacleStateCeres(kdT_, x_cat_, y_cat_, z_cat_, o_p_, n_[0], n_[1], n_[2]);
						double d_ = sqrt((n_[0]-x_cat_)*(n_[0]-x_cat_) + (n_[1]-y_cat_)*(n_[1]-y_cat_) + (n_[2]-z_cat_)*(n_[2]-z_cat_)); 
						dist_obs_cat_ = dist_obs_cat_ + d_;
						ROS_ERROR("Son 2 :  segment=%i , cost_function_obstacles_=%f , d_=%f , m1=%f p_cat=[%f %f %f]",i, cost_function_obstacles_,dist_obs_cat_,m1,x_cat_,y_cat_,z_cat_);
					}
					cost_function_obstacles_ = wf_1 * m1 * dist_obs_cat_;
				}
			}
			else if(obs_in_segment_close_cat.size() > 2 && (stateCat[1] > safety_length) ){
				for (int j = 0; j< obs_in_segment_close_cat.size() ; j++){
					// std::cout <<" j= " << j << " , i= " <<i << " , obs_in_segment_close_cat[j]= "<< obs_in_segment_close_cat[j] << std::endl;
					if (obs_in_segment_close_cat [j] == i){
						count_aux++;
						ROS_ERROR("Son mas de 2 segment=%i", i);
						// std::cout <<" entro= " << count_aux << std::endl;
						break;
					}	 
				}
				if (min_dist_segment[i] > sb_)
					m1 = 0.0;
				else
					m1 = (max_value_residual - min_value_residual)/(0.0 - sb_);
				cost_function_obstacles_ = able_average_segment[i] * wf_1 * m1 * (min_dist_segment[i] - sb_) + cost_function_obstacles_;
				if (count_aux>=obs_in_segment_close_cat.size())
					break;
			}
		}
		residual[0] = cost_function_obstacles_;


		/********** II Constraint to make length Cable longer than distance between UAV and UGV **********/ 

		double min_value_residual1;
		if (stateCat[1] > safety_length){
			m2 = 0.0;
			min_value_residual1 = 0.0;
		}
		else{
			min_value_residual1 = 10.0;
			m2 = (max_value_residual - min_value_residual1)/(safety_length*0.9 - safety_length);
		}
			// m2 = (max_value_residual - min_value_residual)/(1.0 - 0.0);
		residual[1] = wf_2 *( m2 * (stateCat[1] - safety_length) + min_value_residual1);
		// residual[1] = wf_2 * m2 * ( (safety_length-stateCat[1])/safety_length);


		/********** III Constraint to make cable not place below traversable map **********/

		if (0.0 < d_below_z_trav)
			m3 = (max_value_residual - min_value_residual)/(0.1 - 0.0);
		else
			m3 = 0.0;
		residual[2] = wf_3 * m3 * stateCat[1] * (d_below_z_trav);

		std::cout << std::fixed;
		std::cout << std::setprecision(4);
		std::cout << "node[" << statePosUAV[0] << "/" << statePosUGV[0] << "]  , residual[0]= " <<residual[0] << " residual[1]= " <<residual[1] << " residual[2]= " <<residual[2] << " , 		[L=" << stateCat[1]<< "/d=" << safety_length;
		std::cout << "] , min_point_z_cat= [" << min_point_z_cat.x << " , " << min_point_z_cat.y << " , " << min_point_z_cat.z << "] , d_below_z_trav= " << d_below_z_trav << " , pos_in_cat_z_min= " << pos_in_cat_z_min << "/" << points_catenary.size() <<std::endl;

	return true;
   
    }

    double wf_1, wf_2, wf_3, sb_, pr_z_;
	int s_;
	geometry_msgs::Point pos_reel_ugv_;
    double pr_ugv_[3];
    pcl::KdTreeFLANN <pcl::PointXYZ> kdT_;
    pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_;
	Grid3d* g_3D_;
	pcl::KdTreeFLANN <pcl::PointXYZ> kdT_trav_;
    pcl::PointCloud <pcl::PointXYZ>::Ptr pc_trav_;
	Eigen::Vector3d fp_ref_;
	ros::Publisher catenary_marker_pub_;
};

#endif