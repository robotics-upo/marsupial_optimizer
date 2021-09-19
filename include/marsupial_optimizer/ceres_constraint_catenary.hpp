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

#include <octomap_msgs/Octomap.h> 
#include <octomap/OcTree.h>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;


struct CatenaryFunctor {
  CatenaryFunctor(double weight_factor_1, double weight_factor_2, double weight_factor_3, double safty_bound, double length_tether_max_,
  				pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points, Grid3d* grid_3D_,
                pcl::KdTreeFLANN <pcl::PointXYZ> trav_kdT, pcl::PointCloud <pcl::PointXYZ>::Ptr trav_pc, geometry_msgs::Point pos_reel_ugv, int size, 
				double pos_reel_z, Eigen::Vector3d fix_pos_ref, ros::NodeHandlePtr nhP, octomap::OcTree* octotree_full_)
                : wf_1(weight_factor_1), wf_2(weight_factor_2), wf_3(weight_factor_3),sb_(safty_bound), ltm_(length_tether_max_), kdT_(kdT_From_NN), 
				o_p_(obstacles_Points), g_3D_(grid_3D_),kdT_trav_(trav_kdT), pc_trav_(trav_pc), pos_reel_ugv_(pos_reel_ugv),size_(size), 
				pr_z_(pos_reel_z), fp_ref_(fix_pos_ref), o_full_(octotree_full_)
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

		geometry_msgs::Point min_point_z_cat, mid_point_cat, min_point_z_cat1, min_point_z_cat2;
		int pos_in_cat_z_min , pos_in_cat_col_trav1, pos_in_cat_col_trav2, pos_segment_col_trav1, pos_segment_col_trav2;
		bool just_one_axe = bc.configBisection(stateCat[1], pos_init_cat_[0], pos_init_cat_[1], pos_init_cat_[2], statePosUAV[1], statePosUAV[2], statePosUAV[3]);
		bc.getPointCatenary3D(points_catenary);
		bc.getMinPointZCat(min_point_z_cat, pos_in_cat_z_min);

		mP_.markerPoints(catenary_marker_, points_catenary, id_marker_, size_, catenary_marker_pub_);

		n_points_cat_dis = 0;
		std::vector<geometry_msgs::Point> nearest_p_cat_to_obs;
		std::vector<double> min_dist_segment;
		std::vector<double> cat_between_obs;
		nearest_p_cat_to_obs.clear(); min_dist_segment.clear(); cat_between_obs.clear(); 
		int num_point_per_segment = 1;
		int num_segment = ceil(( points_catenary.size() - ( n_points_cat_dis + n_points_cat_dis ) )/num_point_per_segment); // previously (n_points_cat_dis+n_points_cat_dis/2.0)
		int mid_p_cat = ceil(points_catenary.size()/2.0);
		if (num_segment <= 0)
			num_segment = 1;
		for (int i=0 ; i< num_segment ; i++){
			geometry_msgs::Point _p;
			_p.x = -1.0; 
			_p.y = -1.0; 
			_p.z = -1.0; 
			nearest_p_cat_to_obs.push_back(_p);
			min_dist_segment.push_back(0.0);
			cat_between_obs.push_back(0.0);
		}

		// next lines finding closest (d_ < d_safety) catenary points to obstacles and then compute error
		std::vector <int> obs_in_segment_close_cat;
		obs_in_segment_close_cat.clear();
		int count_num_point_segment = 0;
		int count_segment= 0 ;
		bool aux_count = 0;
		int pos_in_segment_z_min = 0;
		double able_average_segment[num_segment];
		able_average_segment[0] = 0.0;
		for(size_t i = 0 ; i < points_catenary.size() ; i++){
			double near_[3];
			if (i >= n_points_cat_dis && (i < points_catenary.size() - n_points_cat_dis) ){
				TrilinearParams d = g_3D_->getPointDistInterpolation((double)points_catenary[i].x, (double)points_catenary[i].y, (double)points_catenary[i].z);
				double x , y, z;
				x = points_catenary[i].x;
				y = points_catenary[i].y;
				z = points_catenary[i].z;
				d_obs_= (d.a0 + d.a1*x + d.a2*y + d.a3*z + d.a4*x*y + d.a5*x*z + d.a6*y*z + d.a7*x*y*z); 
				// Next lines to get nearest obstacle to each segment
				count_num_point_segment++;	
				if (d_obs_ < d_min){
					min_dist_segment[count_segment] = d_obs_;
					d_min = d_obs_;
					nearest_p_cat_to_obs[count_segment].x = points_catenary[i].x; 
					nearest_p_cat_to_obs[count_segment].y = points_catenary[i].y; 
					nearest_p_cat_to_obs[count_segment].z = points_catenary[i].z; 
					if (aux_count && d_obs_ < sb_){
						able_average_segment[count_segment] = 1.0;
						obs_in_segment_close_cat.push_back(count_segment);
						aux_count = false;
						if(i>0){
							octomap::point3d r_;
							octomap::point3d s_(points_catenary[i].x,points_catenary[i].y,points_catenary[i].z); //start for rayCast
							octomap::point3d d_(points_catenary[i].x-points_catenary[i-1].x,
												points_catenary[i].y-points_catenary[i-1].y, 
												points_catenary[i].z-points_catenary[i-1].z ); //direction for rayCast
							bool r_cast_coll = false; 
							r_cast_coll = o_full_->castRay(s_, d_, r_);
							if(r_cast_coll){
								cat_between_obs[i]=1.0;	
								// printf (" [%.1f/%lu] ",cat_between_obs[i],i);
							}
						}
					}	
				}
				if (i == pos_in_cat_z_min)
					pos_in_segment_z_min = count_segment;
				if (i == mid_p_cat){
					mid_point_cat.x = points_catenary[i].x;
					mid_point_cat.y = points_catenary[i].y;
					mid_point_cat.z = points_catenary[i].z;
				}
				if (count_num_point_segment >= num_point_per_segment){
					count_num_point_segment = 0;
					count_segment++;
					aux_count = true;
					d_min = 100.0;
					able_average_segment[count_segment] = 0.0;
				}
			}
		}
		// next lines compute distance between catenary lower point to traversabolity map.
		double below_z_trav[3];
		double d_below_z_trav = 0.0;
		nn.nearestObstacleStateCeres(kdT_trav_ , min_point_z_cat.x, min_point_z_cat.y, min_point_z_cat.z, pc_trav_, below_z_trav[0], below_z_trav[1], below_z_trav[2]);
		if (min_point_z_cat.z <= below_z_trav[2] + sb_){ 
			d_below_z_trav = sqrt((min_point_z_cat.x-below_z_trav[0])*(min_point_z_cat.x-below_z_trav[0])+
								(min_point_z_cat.y-below_z_trav[1])*(min_point_z_cat.y-below_z_trav[1])+
								(min_point_z_cat.z-below_z_trav[2])*(min_point_z_cat.z-below_z_trav[2]));
		}



		// Define min and max bound L 
		double L_, L_max, L_min, L_new;
		double not_catenary_ = 0.0;
		if (!just_one_axe){
			printf("Entro 1\n");
			L_max = L_min = L_ = L_new = safety_length;
			int count_ = 0;
			bool bound_finded_ = false;
			bool starting_in_coll = false;
			int count_point_not_coll = 0;
			while (L_ <= ltm_ || !bound_finded_){
				if (safety_length > 4.0)
						L_ = safety_length * (1.001 + 0.001*count_);
					else
						L_ = safety_length * (1.001 + 0.002*count_);
				if( ltm_ < L_){
					printf("Entro 1.5\n");
					if(!starting_in_coll){
						L_max = L_new; 
						break;
					}
					else{
						L_max = L_min = safety_length;
						not_catenary_ = 1.0;
						ROS_ERROR("NOT SOLUTION FOR CATENARY : Always in collision point [%i]",id_marker_);
						break;
					}
				}
				geometry_msgs::Point p_min;
				double t_min[3];
				int n_;
				bc.configBisection(L_, pos_init_cat_[0], pos_init_cat_[1], pos_init_cat_[2], statePosUAV[1], statePosUAV[2], statePosUAV[3]);
				bc.getPointCatenary3D(points_catenary);
				bc.getMinPointZCat(p_min, n_);
				nn.nearestObstacleStateCeres(kdT_trav_ , p_min.x, p_min.y, p_min.z, pc_trav_, t_min[0], t_min[1], t_min[2]);
				if (p_min.z < t_min[2] ){
					if (starting_in_coll){
						not_catenary_ = 1.0;
						printf("Not catenary foundes: Always in Coliision\n");
						break;
						}
					printf("Entro 1.8 min_point_z_cat[%i]=[%f %f %f]\n",n_, p_min.x, p_min.y, p_min.z);
					L_max = L_new;
					break;
				}
				count_point_not_coll = 0;
				for(size_t i = 0 ; i < points_catenary.size() ; i++){
					if (i >= n_points_cat_dis && (i < points_catenary.size() - n_points_cat_dis) ){
						// if(cat_between_obs[i] == 0.0){ 
						TrilinearParams d = g_3D_->getPointDistInterpolation((double)points_catenary[i].x, 
																			 (double)points_catenary[i].y, 
																			 (double)points_catenary[i].z);
						double x , y, z;
						x = points_catenary[i].x;
						y = points_catenary[i].y;
						z = points_catenary[i].z;
						d_obs_= (d.a0 + d.a1*x + d.a2*y + d.a3*z + d.a4*x*y + d.a5*x*z + d.a6*y*z + d.a7*x*y*z); 
						if (d_obs_ < sb_ ){
							if (count_== 0){
								printf("Entro 2\n");
								starting_in_coll = true;
								break;
							}
							if (!starting_in_coll){
								L_max = L_new; // Is the previous L value
								bound_finded_ = true;
								printf("Entro 2.1\n");
								break;
							}
						}
						else{
							count_point_not_coll++;
						}
					}
					if (count_point_not_coll == points_catenary.size() && starting_in_coll){
						L_min = L_;  // Is the current L value 
						bound_finded_ = true;
						count_point_not_coll = 0;
						printf("Entro 3 :  L_min =%f\n",L_min);
					}
				}
				printf("count_point_not_coll =%i  points_catenary.size()=%lu\n",count_point_not_coll, points_catenary.size());
				L_new = L_;
				if (bound_finded_)
					break;
				count_++;
				printf("count_ =%i  L_=%f L_min=%f L_max=%f \n",count_, L_, L_min, L_max);
			}
			// std::string _y1;
			// std::cout << "First While -> waiting press key : " ;
			// std::cin >> _y1;
			//Just calculate in case catenary start in collision
			if(starting_in_coll && not_catenary_==0.0){
				printf("Entro 4\n");
				L_ = L_min;
				L_new = L_min;
				bound_finded_ = false;
				count_ = 1;
				while (L_ <= ltm_ || !bound_finded_){
					if (L_min > 4.0)
						L_ = L_min * (1.001 + 0.001*count_);
					else
						L_ = L_min * (1.001 + 0.002*count_);
					if( (ltm_ - L_) < 0.0){
						L_max = L_new; 
						printf("Entro 5.0 \n");
						break;
					}
					geometry_msgs::Point p_min;
					double t_min[3];
					int n_;
					bc.configBisection(L_, pos_init_cat_[0], pos_init_cat_[1], pos_init_cat_[2], statePosUAV[1], statePosUAV[2], statePosUAV[3]);
					bc.getPointCatenary3D(points_catenary);
					bc.getMinPointZCat(p_min, n_);
					nn.nearestObstacleStateCeres(kdT_trav_ , p_min.x, p_min.y, p_min.z, pc_trav_, t_min[0], t_min[1], t_min[2]);
					if (p_min.z < t_min[2] ){ 
						L_max = L_new;
						printf("Entro 5.1 min_point_z_cat[%i]=[%f %f %f]\n",n_, p_min.x, p_min.y, p_min.z);
						break;
					}
					for(size_t i = 0 ; i < points_catenary.size() ; i++){
						if (i >= n_points_cat_dis && (i < points_catenary.size() - n_points_cat_dis) ){
							TrilinearParams d = g_3D_->getPointDistInterpolation((double)points_catenary[i].x, (double)points_catenary[i].y, (double)points_catenary[i].z);
							double x , y, z;
							x = points_catenary[i].x;
							y = points_catenary[i].y;
							z = points_catenary[i].z;
							d_obs_= (d.a0 + d.a1*x + d.a2*y + d.a3*z + d.a4*x*y + d.a5*x*z + d.a6*y*z + d.a7*x*y*z); 
							if (d_obs_ < sb_){
								L_max = L_new; 
								printf("Entro 5.2 L_max=%f\n",L_max);
								bound_finded_ = true;
								break;
							}
						}
					}
					L_new = L_;
					if (bound_finded_)
						break;
					count_++;

					printf("count_ =%i  L_=%f L_min=%f L_max=%f \n",count_, L_, L_min, L_max);
					// std::string _y2;
					// std::cout << "Second While -> waiting press key : " ;
					// std::cin >> _y2;
				}
				starting_in_coll = false;
			}
		}
		else{
			ROS_WARN("SOLUTION FOR CATENARY in one axes");
			L_max = L_min = safety_length;
		}	
		
		/********** I Constraint to make cable far away from obstacles **********/
		
		double cost_function_obstacles_ = 0.0;
		// double ave_x_, ave_y_, ave_z_;
		// ave_x_ = ave_y_ = ave_z_ = 0.0;
		// std::vector<geometry_msgs::Point> average_points_coll_;
		// average_points_coll_.clear();
		// bool collision_ = false;
		// int counting = 0;
		// printf ("cat_between_obs: \n");
		int first_coll, last_coll;
		first_coll = last_coll = 0;
		for (int i=0 ; i < num_segment ; i++){
			if (min_dist_segment[i] > sb_)
				m1 = 0.0;
			else
				m1 = (max_value_residual - min_value_residual)/(0.0 - sb_);
			cost_function_obstacles_ = able_average_segment[i] *  m1 * (min_dist_segment[i] - sb_) + cost_function_obstacles_;
			if(cat_between_obs[i]>0.0){
				if (first_coll == 0)
					first_coll  = i;
				last_coll = i;
				// ave_x_ = nearest_p_cat_to_obs[i].x	+ ave_x_;
				// ave_y_ = nearest_p_cat_to_obs[i].y	+ ave_y_;
				// ave_z_ = nearest_p_cat_to_obs[i].z	+ ave_z_;
				// collision_ = true;
				// counting++;
				// printf (" [%.1f/%i] average=[%f %f %f] counting=%i\n", cat_between_obs[i], i, ave_x_, ave_y_, ave_z_, counting);
			}
			// else{
			// 	if(collision_){
			// 		geometry_msgs::Point p_;
			// 		p_.x = ave_x_/counting;
			// 		p_.y = ave_y_/counting;
			// 		p_.z = ave_z_/counting;
			// 		average_points_coll_.push_back(p_);
			// 		collision_ = false;	
			// 		counting = 0;	
			// 		ROS_ERROR("Entro aqui");
			// 	}	
			// }
		}
		// printf ("\n");
		// geometry_msgs::Point middle_point_cat_coll, center_sphere_coll;
		// double is_middle_point = 0.0;
		// middle_point_cat_coll.x = middle_point_cat_coll.y = middle_point_cat_coll.x = 0.0;
		// for(size_t i = 0 ; i < average_points_coll_.size(); i++){
		// 	printf("[%lu]average_points_coll_= [%f %f %f]\n",i+1,average_points_coll_[i].x, average_points_coll_[i].y, average_points_coll_[i].z);
		// 	middle_point_cat_coll.x = average_points_coll_[i].x + middle_point_cat_coll.x;
		// 	middle_point_cat_coll.y = average_points_coll_[i].y + middle_point_cat_coll.y;
		// 	middle_point_cat_coll.z = average_points_coll_[i].z + middle_point_cat_coll.z;
		// }
		// if (average_points_coll_.size() > 0){
		// 	center_sphere_coll.x = middle_point_cat_coll.x/average_points_coll_.size();
		// 	center_sphere_coll.y = middle_point_cat_coll.y/average_points_coll_.size();
		// 	center_sphere_coll.z = middle_point_cat_coll.z/average_points_coll_.size();
		// 	is_middle_point = 1.0;
		// }
			
		residual[0] = wf_1 * (cost_function_obstacles_ + sb_* (last_coll- first_coll) + not_catenary_* sb_ * num_segment);

		//Get plane Catenary
		// std::vector<geometry_msgs::Point> plane;
		// std::vector<geometry_msgs::Point> obs_in_plane;
		// plane.clear(); obs_in_plane.clear();
		// double a1,b1,c1,a2,b2,c2,a ,b ,c ,d ,x3, y3, z3;
		// geometry_msgs::Point lim_p1, lim_p2, lim_p3, lim_p4;
		// if(min_point_z_cat.z < pos_init_cat_[2]){
		// 	x3 = min_point_z_cat.x; 
		// 	y3 = min_point_z_cat.y;
		// 	z3 = min_point_z_cat.z;
		// }
		// else{
		// 	x3 = mid_point_cat.x; 
		// 	y3 = mid_point_cat.y;
		// 	z3 = mid_point_cat.z;	
		// }
		// a1 = statePosUAV[1] - pos_init_cat_[0];
		// b1 = statePosUAV[2] - pos_init_cat_[1];
		// c1 = statePosUAV[3] - pos_init_cat_[2];
		// a2 = x3 - pos_init_cat_[0];
		// b2 = y3 - pos_init_cat_[1];
		// c2 = z3 - pos_init_cat_[2];
		// a = b1 * c2 - b2 * c1;
		// b = a2 * c1 - a1 * c2;
		// c = a1 * b2 - b1 * a2;
		// d = (- a * pos_init_cat_[0] - b * pos_init_cat_[1] - c * pos_init_cat_[2]);
		// if (pos_init_cat_[2] <= statePosUAV[3]){
		// 	lim_p1.x = pos_init_cat_[0]; 	lim_p1.y = pos_init_cat_[1]; 	lim_p1.z = min_point_z_cat.z;
		// 	lim_p2.x = statePosUAV[1]; 		lim_p2.y = statePosUAV[2]; 		lim_p2.z = min_point_z_cat.z;
		// 	lim_p3.x = pos_init_cat_[0]; 	lim_p3.y = pos_init_cat_[1]; 	lim_p3.z = statePosUAV[3];
		// 	lim_p4.x = statePosUAV[1]; 		lim_p4.y = statePosUAV[2];		lim_p4.z = statePosUAV[3];
		// }
		// else{
		// 	lim_p1.x = pos_init_cat_[0]; 	lim_p1.y = pos_init_cat_[1]; 	lim_p1.z = min_point_z_cat.z;
		// 	lim_p2.x = statePosUAV[1]; 		lim_p2.y = statePosUAV[2]; 		lim_p2.z = min_point_z_cat.z;
		// 	lim_p3.x = pos_init_cat_[0]; 	lim_p3.y = pos_init_cat_[1]; 	lim_p3.z = pos_init_cat_[2];
		// 	lim_p4.x = statePosUAV[1]; 		lim_p4.y = statePosUAV[2];		lim_p4.z = pos_init_cat_[2];
		// }
		// double d_limXY = sqrt((lim_p2.x-lim_p1.x)*(lim_p2.x-lim_p1.x)+(lim_p2.y-lim_p1.y)*(lim_p2.y-lim_p1.y));
		// double d_limZ = sqrt((lim_p4.z-lim_p1.z)*(lim_p4.z-lim_p1.z));
		// double step_ = 0.05;
		// double module_ = sqrt((lim_p2.x-lim_p1.x)*(lim_p2.x-lim_p1.x)+(lim_p2.y-lim_p1.y)*(lim_p2.y-lim_p1.y)+(lim_p2.z-lim_p1.z)*(lim_p2.z-lim_p1.z));
		// geometry_msgs::Point point_plane;
		// if (module_ > 0.0){
		// 	double uni_x = (lim_p2.x-lim_p1.x)/module_;
		// 	double uni_y = (lim_p2.y-lim_p1.y)/module_;
		// 	// double uni_z = (lim_p2.z-lim_p1.z)/module_;
		// 	for (int i=0 ; i < d_limZ/step_; i++){
		// 		for(int j=0 ; j < d_limXY/step_ +1 ; j++){ 
		// 			point_plane.x = lim_p1.x + uni_x*step_*j;
		// 			point_plane.y = lim_p1.y + uni_y*step_*j;
		// 			point_plane.z = lim_p1.z + 1.0*step_*i;
		// 			double grid_dist_obs = 0.0;
		// 			if (g_3D_->isIntoMap(point_plane.x,point_plane.y,point_plane.z));
		// 				grid_dist_obs = g_3D_->getPointDist(point_plane.x,point_plane.y,point_plane.z);
		// 			if(grid_dist_obs < 0.05)
		// 				obs_in_plane.push_back(point_plane);
		// 			else
		// 				plane.push_back(point_plane);
		// 		}
		// 	}
		// }
		// else{
		// 	// double uni_z = sqrt((lim_p4.z-lim_p1.z)*(lim_p4.z-lim_p1.z));
		// 	double uni_z = 1.0;
		// 	for (int i=0 ; i < d_limZ/step_ ; i++){
		// 		point_plane.x = lim_p1.x ;
		// 		point_plane.y = lim_p1.y ;
		// 		point_plane.z = lim_p1.z + uni_z*step_*i;
		// 		double grid_dist_obs = 0.0;
		// 		if (g_3D_->isIntoMap(point_plane.x,point_plane.y,point_plane.z));
		// 			grid_dist_obs = g_3D_->getPointDist(point_plane.x,point_plane.y,point_plane.z);
		// 		if(grid_dist_obs < 0.05)
		// 			obs_in_plane.push_back(point_plane);
		// 		else
		// 			plane.push_back(point_plane);
		// 	}
		// }

		// mP_.markerPoints(plane_marker_, plane, 0, size_, plane_pub_, 1);
		// mP_.markerPoints(obs_plane_marker_, obs_in_plane, 0, size_, obs_plane_pub_, 2);



		// std::string _y;
		// std::cout << "waiting press key : " ;
		// std::cin >> _y;
		/********** II Constraint to make length Cable longer than distance between UAV and UGV **********/ 

		double min_value_residual1, cost1_, cost2_;
		cost1_ = cost2_ = 0.0;
		if (not_catenary_ == 1.0){
			min_value_residual1 = 10.0;
			m2 = (max_value_residual - min_value_residual1)/(safety_length*1.1 - safety_length);
			cost1_ = cost2_ = 1.0;
		}
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
		else{
			m2 = 0.0;
			min_value_residual1 = 0.0;
			cost1_ = cost2_ = 0.0;
		}
		
		residual[1] = wf_2 * ( cost1_* (1.0/stateCat[1]) *( m2 * (stateCat[1] - safety_length) + min_value_residual1) + 
							   cost2_* ( stateCat[1] ) * ( m2 * (stateCat[1] - safety_length) + min_value_residual1) );


		/********** III Constraint to make cable not place below traversable map **********/

		if (0.0 < d_below_z_trav)
			m3 = (max_value_residual - min_value_residual)/(0.1 - 0.0);
		else
			m3 = 0.0;
		residual[2] = wf_3 * m3 * stateCat[1] * (d_below_z_trav);


		std::cout << std::fixed;
		std::cout << std::setprecision(4);
		std::cout << "node[" << statePosUAV[0] << "/" << statePosUGV[0] << "]  , residual[0]= " <<residual[0] << " residual[1]= " <<residual[1] << " residual[2]= " <<residual[2]; 
		std::cout << " , 		[L=" << stateCat[1]<< "/Lmin=" << L_min << "/Lmax=" << L_max << "/d=" << safety_length <<" , points_cat_in_coll=["
				  <<first_coll<<"-"<<last_coll<<"/"<<last_coll-first_coll<<"/"<<num_segment<<"]"<<std::endl;
		// std::cout << "] , min_point_z_cat= [" << min_point_z_cat.x << " , " << min_point_z_cat.y << " , " << min_point_z_cat.z << "] , d_below_z_trav= " << d_below_z_trav;
		// std::cout << " ,   obs_segment_close_cat = [" << obs_in_segment_close_cat.size()<<"/"<<num_segment<<"]" << std::endl;


		// std::string _y2;
		// std::cout << "Continue While -> waiting press key : " ;
		// std::cin >> _y2;
	return true;
    }

    double wf_1, wf_2, wf_3, sb_, pr_z_, ltm_;
	int size_;
	geometry_msgs::Point pos_reel_ugv_;
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