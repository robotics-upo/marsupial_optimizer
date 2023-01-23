#ifndef CERES_CONSTRAINS_CATENARY_OBSTACLE_AUTODIFF_HPP
#define CERES_CONSTRAINS_CATENARY_OBSTACLE_AUTODIFF_HPP

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

#include "catenary_checker/near_neighbor.hpp"
#include "catenary_checker/bisection_catenary_3D.h"
#include "misc/marker_publisher.h"

#include <octomap_msgs/Octomap.h> 
#include <octomap/OcTree.h>

#include <iostream>
#include <fstream>
#include <string>

using ceres::AutoDiffCostFunction;
using ceres::NumericDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;


class AutodiffCatenaryFunctor {

public:
    AutodiffCatenaryFunctor(){}

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

	struct BisectionCatenary 
	{
		BisectionCatenary (ros::NodeHandlePtr nhP_, Grid3d* g_3D_, octomap::OcTree* o_full_, double sb_, pcl::KdTreeFLANN <pcl::PointXYZ> kdT_, 
		pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_, pcl::KdTreeFLANN <pcl::PointXYZ> kdT_trav_, pcl::PointCloud <pcl::PointXYZ>::Ptr pc_trav_, int size_, float map_resolution, bool use_dist_func_)
		: nhP(nhP_), g_3D(g_3D_), o_full(o_full_), sb(sb_), kdT(kdT_), o_p(o_p_), kdT_trav(kdT_trav_), pc_trav(pc_trav_), size(size_), m_r_(map_resolution), use_dist_func(use_dist_func_)
		{
			catenary_marker_pub_ = nhP_->advertise<visualization_msgs::MarkerArray>("catenary_marker", 1);
			plane_pub_ = nhP_->advertise<visualization_msgs::MarkerArray>("plane_markerArray", 1);
			obs_plane_pub_ = nhP_->advertise<visualization_msgs::MarkerArray>("obsplane_markerArray", 1);
		}
		bool operator()(const double *state1,const double *state2, const double *state2f,const double *Length, double *Catenary) const 
		{
			MarkerPublisher mP_;
			bisectionCatenary bc(nhP);
			NearNeighbor nn;

			visualization_msgs::MarkerArray catenary_marker_, plane_marker_, obs_plane_marker_;
			std::vector<geometry_msgs::Point> points_catenary;
			std::vector<double> dist_obst_cat; 
			std::vector<int> cat_between_obs, pos_cat_in_coll;
			geometry_msgs::Point min_point_z_cat;

			int id_marker_ = state2[0];		// Related with Marker frame.id
			double min_val_proximity_ = 0.04;
			// double min_val_proximity_ = sb;
			double cost_cat = 0.0;
			int first_coll, last_coll;

			points_catenary.clear();  dist_obst_cat.clear(); pos_cat_in_coll.clear(); cat_between_obs.clear(); 
			bc.readDataForCollisionAnalisys(g_3D, sb, o_full, kdT_trav, pc_trav);
			bool just_one_axe = bc.configBisection(Length[1], state1[1], state1[2], state1[3], state2[1], state2[2], state2[3], true);
			bc.getPointCatenary3D(points_catenary);
			bc.getStatusCollisionCat(dist_obst_cat, pos_cat_in_coll, cat_between_obs, first_coll, last_coll);
			double min_distance_cat_obs_ = bc.min_distance_cat_obs;

			mP_.markerPoints(catenary_marker_, points_catenary, id_marker_, size, catenary_marker_pub_);
			if (bc.L_minor_than_D == true){
				std::string yy_ ;
				yy_ = "s";
				/********************* To obligate stop method and check Optimization result *********************/
				std::cout << " Press key 'y': ";
				while (yy_ != "y"){
					std::cin >> yy_ ;
				}
			}

			if (( (state2f[1]-state2[1]) != 0.0 || (state2f[2]-state2[2]) != 0.0 || (state2f[3]-state2[3]) != 0.0) && first_coll!= 0){
				octomap::point3d r_;
				octomap::point3d s_(state2f[1], state2f[2], state2f[3]); //start for rayCast
				octomap::point3d d_(state2[1]-state2f[1] , state2[2]-state2f[2] , state2[3]-state2f[3]); //direction for rayCast
				bool r_cast_coll = false; 
				r_cast_coll =  o_full->castRay(s_, d_, r_);
				double dist_b_uav = sqrt(pow(state2[1]-s_.x(),2)+pow(state2[2]-s_.y(),2)+pow(state2[3]-s_.z(),2));
				double distObs_ = sqrt ((s_.x()-r_.x())*(s_.x()-r_.x())+(s_.y()-r_.y())*(s_.y()-r_.y())+(s_.z()-r_.z())*(s_.z()-r_.z()) );
				if(r_cast_coll && distObs_ <= dist_b_uav )
					last_coll = points_catenary.size();
			}

			double x_, y_, z_, cost_ , add_extracost;
			double d_below_z_trav = 0.0;
			geometry_msgs::Point n_coll_cat, point_coll_trav;
			geometry_msgs::Vector3 p_;
			int is_collision = 0;
			int count_coll = 0;
			for (size_t i = 0; i < points_catenary.size(); i++){
				p_.x = m_r_ * round( (points_catenary[i].x) * 1/m_r_);
				p_.y = m_r_ * round( (points_catenary[i].y) * 1/m_r_);
				p_.z = m_r_ * round( (points_catenary[i].z) * 1/m_r_);
				// bool _is_into_ = g_3D->isIntoMap((double)p_.x, (double)p_.y, (double)p_.z);
				double d_obs;
				if(use_dist_func){
					bool is_into_ = g_3D->isIntoMap((double)p_.x,(double)p_.y,(double)p_.z);
					if (is_into_){
						TrilinearParams d = g_3D->getPointDistInterpolation((double)p_.x, (double)p_.y, (double)p_.z);
						x_ = p_.x;
						y_ = p_.y;
						z_ = p_.z;
						d_obs= (d.a0 + d.a1*x_ + d.a2*y_ + d.a3*z_ + d.a4*x_*y_ + d.a5*x_*z_ + d.a6*y_*z_ + d.a7*x_*y_*z_);
					}
					else
						d_obs = -1.0;
				}
				else{
					geometry_msgs::Vector3 near_;
            		nn.nearestObstacleStateCeres(kdT , p_.x,p_.y,p_.z, o_p, near_.x, near_.y, near_.z);
					d_obs = sqrt(pow(near_.x-p_.x,2) + pow(near_.y-p_.y,2) + pow(near_.z-p_.z,2));
				}

				if((i >= first_coll && i <= last_coll && i > 0) ){
					cost_ = (1.0/min_val_proximity_)*2.0;
					is_collision = 1;
					count_coll++;
				}
				else if (d_obs < sb){ 
					if (d_obs < min_val_proximity_)
						d_obs = min_val_proximity_;
					// cost_ = (1.0/d_obs)*1.0;
					cost_ = (1.0/min_distance_cat_obs_)*2.0;
					// cost_ = (1.0/min_val_proximity_)*1.0;
					is_collision = 2;
					count_coll ++;
				}
				else
					cost_ = (1.0/d_obs)*1.0;
					
				cost_cat = cost_cat + cost_;
			}

			// if (is_collision==1){
			// 	for(int k = 0 ; k < bc.dist_obst_cat.size(); k++){
			// 		printf("dist_obst_cat[%i/%lu]= %f \n", k,dist_obst_cat.size(),dist_obst_cat[k]);
			// 	}
			// 	std::string yy_ ;
			// 	yy_ = "s";
			// 	std::cout << "Cat[" << state1[0] <<"] in collision["<<first_coll << "-"<<last_coll  << "] cost="<< cost_cat <<". Press key 'y': ";
			// 	while (yy_ != "y"){
			// 		std::cin >> yy_ ;
			// 	}
			// }
			// if (is_collision==2){
			// 	for(int k = 0 ; k < bc.dist_obst_cat.size(); k++){
			// 		printf("dist_obst_cat[%i/%lu]= %f \n", k,dist_obst_cat.size(),dist_obst_cat[k]);
			// 	}
			// 	std::string yy_ ;
			// 	yy_ = "s";
			// 	std::cout << "Cat[" << state1[0] <<"] close obstacle["<<first_coll << "-"<<last_coll  << "] cost="<< cost_cat <<". Press key 'y': ";
			// 	while (yy_ != "y"){
			// 		std::cin >> yy_ ;
			// 	}
			// }

			Catenary[0] = cost_cat;
			Catenary[1] = first_coll;
			Catenary[2] = last_coll;
			// Catenary[3] = last_coll-first_coll;
			Catenary[3] = count_coll;
			Catenary[4] = points_catenary.size();
			Catenary[5] = bc.num_point_per_unit_length;

			return true;
		}
		ros::Publisher catenary_marker_pub_ , plane_pub_, obs_plane_pub_;
		ros::NodeHandlePtr nhP;

		octomap::OcTree* o_full;
		bool use_dist_func;
		float m_r_;
		double sb;
		int size;
		pcl::KdTreeFLANN <pcl::PointXYZ> kdT, kdT_trav;
		pcl::PointCloud <pcl::PointXYZ>::Ptr o_p, pc_trav;
		Grid3d* g_3D;
	};

	struct CatenaryFunctor 
	{
	CatenaryFunctor(double weight_factor_1, double weight_factor_3, double safty_bound, float min_length_cat, double length_tether_max_,
					pcl::KdTreeFLANN <pcl::PointXYZ> kdT_From_NN, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_Points, Grid3d* grid_3D_,
					pcl::KdTreeFLANN <pcl::PointXYZ> trav_kdT, pcl::PointCloud <pcl::PointXYZ>::Ptr trav_pc, geometry_msgs::Vector3 pos_reel_ugv, int size, 
					geometry_msgs::Vector3 fix_pos_ref, ros::NodeHandlePtr nhP_, octomap::OcTree* octotree_full_, float map_resolution, bool write_data, 
					bool use_dist_func_, std::string user_name)
					: wf_1(weight_factor_1), sb_(safty_bound), m_L_c_(min_length_cat), kdT_(kdT_From_NN), 
					o_p_(obstacles_Points), g_3D_(grid_3D_),kdT_trav_(trav_kdT), pc_trav_(trav_pc), pos_reel_ugv_(pos_reel_ugv),size_(size), 
					fp_ref(fix_pos_ref), o_full_(octotree_full_), m_r_(map_resolution), w_d_(write_data), use_dist_func(use_dist_func_), user_(user_name)
		{
			nhP = nhP_;
			
			ray_casting.reset(new ceres::CostFunctionToFunctor<1,4,4>(
							  new ceres::NumericDiffCostFunction<RayCasting, ceres::CENTRAL,1,4,4>(new RayCasting(o_full_))));

			bisection_catenary.reset(new ceres::CostFunctionToFunctor<6,4,4,4,2>(
							  new ceres::NumericDiffCostFunction<BisectionCatenary, ceres::CENTRAL,6,4,4,4,2>
							  (new BisectionCatenary(nhP_, g_3D_, o_full_, sb_, kdT_, o_p_, kdT_trav_, pc_trav_, size_, m_r_, use_dist_func))));
		}

		template <typename T>
		bool operator()(const T* const stateUGV, const T* const stateUAV, const T* const stateCat, T* residual) const 
		{
			/*----------------------  Define Variables  ----------------------*/

			T pr_ugv_ [3];
			T L_, d_obs_[1];

			pr_ugv_[0] = T{pos_reel_ugv_.x};
			pr_ugv_[1] = T{pos_reel_ugv_.y};
			pr_ugv_[2] = T{pos_reel_ugv_.z};

			const T ugv_reel[4] = {stateUGV[0],stateUGV[1],stateUGV[2],stateUGV[3] + pr_ugv_[2]};
			const T stateUAV_[4] = {stateUAV[0], stateUAV[1], stateUAV[2], stateUAV[3]};
			const T uav_fix[4] = {stateUAV[0], T{fp_ref.x}, T{fp_ref.y}, T{fp_ref.z}};

			T dist  = T{ 1.005 * sqrt(pow(stateUAV[1]-ugv_reel[1],2)+pow(stateUAV[2]-ugv_reel[2],2)+pow(stateUAV[3]-ugv_reel[3],2))}; 
			if (stateCat[1] < dist )
				L_ = dist;
			else
				L_ = stateCat[1];

			const T Length_[2] = {stateCat[0] , L_};
			/*----------------------  Compute Ray Casting for LoS between reel-UGV and UAV to get L----------------------*/

            T kind_length_initial[1];
			if ((ugv_reel[1]-stateUAV[1]) != 0.0 || (ugv_reel[2]-stateUAV[2]) != 0.0 || (ugv_reel[3]-stateUAV[3]) != 0.0)
				(*ray_casting)(ugv_reel, stateUAV_, kind_length_initial);

			/*----------------------  Compute Catenary Bisection Method  ----------------------*/
			
			T catenary[6];
			(*bisection_catenary)(ugv_reel, stateUAV_, uav_fix, Length_, catenary);

			/********************************   Compute Residual   ********************************/

			T cost_cat_ = catenary[0];
			// residual[0] = wf_1 * (cost_cat_/(catenary[4]-catenary[3]));
			// residual[0] = wf_1 * (cost_cat_/(L_-catenary[3]/catenary[5]));
			residual[0] = wf_1 * (cost_cat_);

			if(w_d_){
				std::ofstream ofs;
				std::string name_output_file = "/home/"+user_+"/residuals_optimization_data/catenary.txt";
				ofs.open(name_output_file.c_str(), std::ofstream::app);
				if (ofs.is_open()) 
					ofs << residual[0] << " / "
						<< std::endl;
				ofs.close();
				std::ofstream ofs2;
				std::string name_output_file2 = "/home/"+user_+"/residuals_optimization_data/catenary2.txt";
				ofs2.open(name_output_file2.c_str(), std::ofstream::app);
				if (ofs2.is_open()) 
					ofs2 << residual[0] << " ; "
						<< "(" << stateUGV[0] << "); "
						<< "L= " << stateCat[1] << "; "
						<< "L_s= " << Length_[1] << "; " 
						<< "d= " << dist << "; " 
						<< "cat_coll=" << catenary[1] << "-" << catenary[2] << "-" << catenary[3] << "; "
						<< "cost_cat_=" << cost_cat_ << "; "
						// << "N=" << cost_cat_/(catenary[4]-catenary[3]) << "; "
						<< "N=" << cost_cat_/(L_-catenary[3]/catenary[5]) << "; "
						// << "y_" << y_ << "; "
						<< "cat.size= " << catenary[4]  << "; "
						<< "num_point_per_unit_length= " << catenary[5]  << "; "
						<< " is_LoS= " << kind_length_initial[0] <<";" 
						<< "posUGV=" << stateUGV[1] << "-" << stateUGV[2] << "-" << stateUGV[3] << "-"<<";"
						<< "posUAV=" << stateUAV[1] << "-" << stateUAV[2] << "-" << stateUAV[3] << "-"<<"/"
						<< std::endl;
				ofs2.close();
			}
		
			// if (catenary[3] > T{0.0}){
			// 	std::string yy_ ;
			// 	yy_ = "s";
			// 	/********************* To obligate stop method and check Optimization result *********************/
			// 		std::cout << " Press key 'y': ";
			// 		while (yy_ != "y"){
			// 			std::cin >> yy_ ;
			// 		}
			// 	/**********************************************************************************************/
			// }
			
		return true;
		}
		
        std::unique_ptr<ceres::CostFunctionToFunctor<1,4,4> > ray_casting;
        std::unique_ptr<ceres::CostFunctionToFunctor<6,4,4,4,2> > bisection_catenary;

		bool w_d_, use_dist_func;
		double wf_1;
		float m_L_c_, m_r_;
		int size_;
		geometry_msgs::Vector3 pos_reel_ugv_, fp_ref;
		std::string user_;

		octomap::OcTree* o_full_;
		double sb_;
		pcl::KdTreeFLANN <pcl::PointXYZ> kdT_, kdT_trav_;
		pcl::PointCloud <pcl::PointXYZ>::Ptr o_p_ , pc_trav_;
		Grid3d* g_3D_;
		ros::NodeHandlePtr nhP;
	};

private:

};

#endif