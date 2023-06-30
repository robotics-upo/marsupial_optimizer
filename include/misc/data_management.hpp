#ifndef DATA_MANAGEMENT_HPP
#define DATA_MANAGEMENT_HPP

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include "catenary_solver_ceres.hpp"

#include <ros/ros.h>
#include <geometry_msgs/Point.h> 
#include <geometry_msgs/Vector3.h> 

#include "Eigen/Core"

#include <pcl-1.10/pcl/point_cloud.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <octomap_msgs/Octomap.h> 
#include <octomap/OcTree.h>

#include "catenary_checker/grid3d.hpp"
#include "catenary_checker/bisection_catenary_3D.h"
#include "catenary_checker/near_neighbor.hpp"

class DataManagement
{
	public:
		DataManagement();
		// ~DataManagement(){};
		virtual void initDataManagement(std::string path_, std::string name_output_file_, int scenario_number_, int num_pos_initial_,double initial_velocity_ugv_, 
							double initial_velocity_uav_, double initial_acceleration_ugv_, double initial_acceleration_uav_, double bound_cat_obs_, 
							geometry_msgs::Vector3 pos_reel_ugv_, std::vector<geometry_msgs::Vector3> vec_pose_init_ugv_, std::vector<geometry_msgs::Vector3> vec_pose_init_uav_,	
							std::vector<float> vec_len_cat_init_, std::vector<geometry_msgs::Quaternion> vec_rot_ugv_, 
							std::vector<geometry_msgs::Quaternion> vec_rot_uav_, octomap::OcTree* octree_full_,octomap::OcTree* octree_ugv_, Grid3d* grid_3D_);
		virtual void writeTemporalDataBeforeOptimization(std::vector<double> vec_dist_init_ugv_, std::vector<double> vec_dist_init_uav_, 
									std::vector<double> vec_time_init_, std::vector<double> v_angles_kinematic_ugv, std::vector<double> v_angles_kinematic_uav_);
		void writeTemporalDataAfterOptimization(int _s, std::vector<geometry_msgs::Vector3> vec_pose_ugv_opt_, std::vector<geometry_msgs::Vector3> vec_pose_uav_opt_, 
												std::vector<double> vec_time_opt_, std::vector<float> vec_len_cat_opt_,
												std::vector<geometry_msgs::Quaternion> vec_rot_ugv_, std::vector<geometry_msgs::Quaternion> vec_rot_uav_, 
												std::vector<double> v_angles_kinematic_ugv_, std::vector<double> v_angles_kinematic_uav_);
		virtual void getDataForOptimizerAnalysis(pcl::KdTreeFLANN <pcl::PointXYZ> kdt_, pcl::KdTreeFLANN <pcl::PointXYZ> kdt_all_, 
												 pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_points_ , pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_points_all_, 
												 double opt_compute_time_ , std::string mode_, int &n_coll_init_path_, int &n_coll_opt_traj_);
 		virtual geometry_msgs::Vector3 getReelPos(const float px_, const float py_, const float pz_,const float qx_, const float qy_, const float qz_, const float qw_, geometry_msgs::Vector3 p_reel_);
		virtual geometry_msgs::Vector3 getEulerAngles(const float qx_, const float qy_, const float qz_, const float qw_);
		virtual bool isObstacleBetweenTwoPoints(geometry_msgs::Vector3 pose_opt_1, geometry_msgs::Vector3 pose_opt_2, bool oct_full_);
		virtual inline void feasibilityAnalisysTrajectory(float init_cost, float final_cost, float succes_steps, float unsuccess_step, float time_opt, int &n_coll_opt_cat_);
		// virtual void writeCatenaryFailData(int num_cat_fail_, double pos_node_, double old_length_, double dist_, double new_length_);
		// virtual void openWriteCatenaryFailData();
		// virtual void closeWriteCatenaryFailData();

		std::string path;
		std::string output_file, name_output_file;
		int scenario_number, num_pos_initial;
		std::ofstream ofs, feasibility;
		std::ofstream file_in_time ,file_in_velocity, file_in_acceleration, file_in_rotation, file_in_kinematic;
		std::ofstream file_out_time, file_out_velocity, file_out_acceleration, file_out_rotation, file_out_kinematic ;
		std::ofstream catenary_data_out;

		std::vector<geometry_msgs::Vector3> vec_pose_ugv_opt, vec_pose_uav_opt;
		// std::vector<double> vec_time_ugv_opt, vec_time_uav_opt;
		std::vector<double> vec_time_opt;
		std::vector<double> vec_dist_ugv_opt, vec_dist_uav_opt, vec_vel_ugv_opt, vec_vel_uav_opt, vec_acc_ugv_opt, vec_acc_uav_opt;
		std::vector<geometry_msgs::Vector3> vec_pose_init_ugv, vec_pose_init_uav;

		double initial_velocity_ugv, initial_velocity_uav, initial_acceleration_ugv, initial_acceleration_uav, bound_cat_obs;

		std::vector<double> vec_dist_init_ugv, vec_dist_init_uav;
		// std::vector<double> vec_time_init_ugv, vec_time_init_uav;
		std::vector<double> vec_time_init;
		std::vector<float> vec_len_cat_init, vec_len_cat_opt;
		std::vector<geometry_msgs::Quaternion> vec_init_rot_ugv, vec_init_rot_uav, vec_opt_rot_ugv, vec_opt_rot_uav;

	    NearNeighbor nn_;

		geometry_msgs::Point ugv_pos_catenary;
		geometry_msgs::Vector3 pos_reel_ugv;

		octomap::OcTree* octree_full;
		octomap::OcTree* octree_ugv;

		int num_coll_ugv_traj, num_coll_uav_traj, num_points_coll_cat, num_cat_coll, num_cat_fail_lmin, num_cat_fail_lmax;
		std::string mode , pos_coll_cat;

		Grid3d* g_3D_;

	protected:

	private:

};

inline DataManagement::DataManagement()
{
	num_coll_ugv_traj = num_coll_uav_traj = num_points_coll_cat = num_cat_coll = num_cat_fail_lmin = num_cat_fail_lmax = 0; 
	pos_coll_cat = "";
}

inline void DataManagement::initDataManagement(std::string path_, std::string name_output_file_, int scenario_number_, int num_pos_initial_,
				double initial_velocity_ugv_, double initial_velocity_uav_, double initial_acceleration_ugv_, double initial_acceleration_uav_, double bound_cat_obs_, 
				geometry_msgs::Vector3 pos_reel_ugv_, std::vector<geometry_msgs::Vector3> vec_pose_init_ugv_, std::vector<geometry_msgs::Vector3> vec_pose_init_uav_, 
				std::vector<float> vec_len_cat_init_, std::vector<geometry_msgs::Quaternion> vec_rot_ugv_, std::vector<geometry_msgs::Quaternion> vec_rot_uav_, 
				octomap::OcTree* octree_full_, octomap::OcTree* octree_ugv_, Grid3d* grid_3D_)
{
	path = path_;
	name_output_file = name_output_file_;
	scenario_number = scenario_number_; 
	num_pos_initial = num_pos_initial_;

	initial_velocity_ugv = initial_velocity_ugv_; 
	initial_velocity_uav = initial_velocity_uav_; 
	initial_acceleration_ugv = initial_acceleration_ugv_; 
	initial_acceleration_uav = initial_acceleration_uav_;

	vec_pose_init_ugv.clear();
	vec_pose_init_uav.clear();
	vec_len_cat_init.clear(); 
	vec_pose_init_ugv = vec_pose_init_ugv_;
	vec_pose_init_uav = vec_pose_init_uav_;
	vec_len_cat_init = vec_len_cat_init_;
	vec_init_rot_ugv = vec_rot_ugv_;
	vec_init_rot_uav = vec_rot_uav_;
	bound_cat_obs = bound_cat_obs_;
	
	pos_reel_ugv = pos_reel_ugv_;

	octree_full = octree_full_;
	octree_ugv = octree_ugv_;

	g_3D_ = grid_3D_;

	output_file = path+"results"+"_stage_"+std::to_string(scenario_number)+"_InitPos_"+std::to_string(num_pos_initial)+"_"+name_output_file;
}

inline void DataManagement::writeTemporalDataBeforeOptimization(std::vector<double> vec_dist_init_ugv_, std::vector<double> vec_dist_init_uav_, 
					std::vector<double> vec_time_init_, std::vector<double> v_angles_kinematic_ugv_, std::vector<double> v_angles_kinematic_uav_)
{
	vec_dist_init_ugv.clear(); 
	vec_dist_init_uav.clear(); 
	vec_time_init.clear();
	vec_dist_init_ugv = vec_dist_init_ugv_;
	vec_dist_init_uav = vec_dist_init_uav_;
	vec_time_init = vec_time_init_;

	//! Save temporal state before optimization
	file_in_time.open (path+"results"+"_stage_"+std::to_string(scenario_number)+"_InitPos_"+std::to_string(num_pos_initial)+"_initial_time_ugv-uav.txt", std::ofstream::app);
	file_in_velocity.open (path+"results"+"_stage_"+std::to_string(scenario_number)+"_InitPos_"+std::to_string(num_pos_initial)+"_initial_velocity_ugv-uav.txt", std::ofstream::app);
	file_in_acceleration.open (path+"results"+"_stage_"+std::to_string(scenario_number)+"_InitPos_"+std::to_string(num_pos_initial)+"_initial_acceleration_ugv-uav.txt", std::ofstream::app);
	file_in_rotation.open (path+"results"+"_stage_"+std::to_string(scenario_number)+"_InitPos_"+std::to_string(num_pos_initial)+"_initial_rotation_ugv-uav.txt", std::ofstream::app);
	file_in_kinematic.open (path+"results"+"_stage_"+std::to_string(scenario_number)+"_InitPos_"+std::to_string(num_pos_initial)+"_initial_kinematic_ugv-uav.txt", std::ofstream::app);
	
	double _sum_dist_ugv = 0.0;
	double _sum_time_ugv = 0.0;
	double _sum_dist_uav = 0.0;
	double _sum_time_uav = 0.0;
	double _value_vel_ugv = 0.0;
	double _value_vel_uav = 0.0;
	double _value_acc_ugv = 0.0;
	double _value_acc_uav = 0.0;

	// for (size_t i = 0; i < vec_time_init_uav.size() - 1; i++){
	for (size_t i = 0; i < vec_time_init.size() - 1; i++){
		// Conditions for UGV
		_sum_time_ugv = _sum_time_ugv + vec_time_init[i+1];
		if ( vec_dist_init_ugv[i] < 0.0001){
			_sum_dist_ugv = _sum_dist_ugv + 0.0;
			_value_vel_ugv = 0.0;
		}
		else{
			_sum_dist_ugv = _sum_dist_ugv + vec_dist_init_ugv[i];
			_value_vel_ugv = initial_velocity_ugv;
		}
		// Conditions for UAV 
		_sum_time_uav = _sum_time_uav + vec_time_init[i+1];
		if ( vec_dist_init_uav[i] < 0.0001){
			_sum_dist_uav = _sum_dist_uav + 0.0;
			_value_vel_uav = 0.0;
		}
		else{
			_sum_dist_uav = _sum_dist_uav + vec_dist_init_uav[i];
			_value_vel_uav = initial_velocity_uav;
		}
		
		file_in_time << std::setprecision(6) << _sum_dist_ugv << ";" << _sum_time_ugv << ";" << _sum_dist_uav << ";" << _sum_time_uav << "/";
		file_in_velocity  << std::setprecision(6) << _sum_dist_ugv << ";" << _value_vel_ugv << ";" << _sum_dist_uav << ";" << _value_vel_uav << "/";
		if (i > 0){
			// Conditions for Acc UGV
			if( (vec_dist_init_ugv[i] - vec_dist_init_ugv[i-1] < 0.0001) && (vec_dist_init_ugv[i+1] - vec_dist_init_ugv[i] < 0.0001) )
				_value_acc_ugv = 0.0;
			else
				_value_acc_ugv = initial_acceleration_ugv;
			// Conditions for Acc UGV
			if( (vec_dist_init_uav[i] - vec_dist_init_uav[i-1] < 0.0001) && (vec_dist_init_uav[i+1] - vec_dist_init_uav[i] < 0.0001) )
				_value_acc_uav = 0.0;
			else
				_value_acc_uav = initial_acceleration_uav;
			file_in_acceleration << std::setprecision(6) << _sum_dist_ugv << ";" << _value_acc_ugv << ";" << _sum_dist_uav << ";" << initial_acceleration_uav << "/";
		}
		geometry_msgs::Vector3 _rot_init_ugv = getEulerAngles(vec_init_rot_ugv[i].x,vec_init_rot_ugv[i].y,vec_init_rot_ugv[i].z,vec_init_rot_ugv[i].w);
		geometry_msgs::Vector3 _rot_init_uav = getEulerAngles(vec_init_rot_uav[i].x,vec_init_rot_uav[i].y,vec_init_rot_uav[i].z,vec_init_rot_uav[i].w);

		file_in_rotation  << std::setprecision(6) << _rot_init_ugv.x << ";" << _rot_init_ugv.y << ";" << _rot_init_ugv.z << ";" 
												  << _rot_init_uav.x << ";" << _rot_init_uav.y << ";" << _rot_init_uav.z << "/";
	}

	for(size_t i=0 ; i< v_angles_kinematic_ugv_.size() ; i++){
		file_in_kinematic << i << ";" << v_angles_kinematic_ugv_[i] << ";" << v_angles_kinematic_uav_[i] << "/";
	}

	file_in_time << std::endl;
	file_in_time.close();
	file_in_velocity << std::endl;
	file_in_velocity.close();
	file_in_acceleration << std::endl;
	file_in_acceleration.close();
	file_in_rotation << std::endl;
	file_in_rotation.close();
	file_in_kinematic << std::endl;
	file_in_kinematic.close();
}

inline void DataManagement::writeTemporalDataAfterOptimization(int _s, std::vector<geometry_msgs::Vector3> vec_pose_ugv_opt_, 
					std::vector<geometry_msgs::Vector3> vec_pose_uav_opt_, std::vector<double> vec_time_opt_, 
					std::vector<float> vec_len_cat_opt_, std::vector<geometry_msgs::Quaternion> vec_rot_ugv_, 
					std::vector<geometry_msgs::Quaternion> vec_rot_uav_, std::vector<double> v_angles_kinematic_ugv_, std::vector<double> v_angles_kinematic_uav_)
{
	vec_pose_ugv_opt.clear();
	vec_pose_uav_opt.clear();
	// vec_time_ugv_opt.clear();
	// vec_time_uav_opt.clear();
	vec_time_opt.clear();
	vec_len_cat_opt.clear();
	vec_pose_ugv_opt = vec_pose_ugv_opt_;
	vec_pose_uav_opt = vec_pose_uav_opt_;
	// vec_time_ugv_opt = vec_time_ugv_opt_;
	// vec_time_uav_opt = vec_time_uav_opt_;
	vec_time_opt = vec_time_opt_;
	vec_len_cat_opt = vec_len_cat_opt_;
	vec_opt_rot_ugv = vec_rot_ugv_;
	vec_opt_rot_uav = vec_rot_uav_;
	
	vec_dist_ugv_opt.clear(); 
	vec_dist_uav_opt.clear(); 
	vec_vel_ugv_opt.clear(); 
	vec_vel_uav_opt.clear(); 
	vec_acc_ugv_opt.clear();
	vec_acc_uav_opt.clear();

	//! Save Temporal Data Optimized in File.txt 
	file_out_time.open (path+"results"+"_stage_"+std::to_string(scenario_number)+"_InitPos_"+std::to_string(num_pos_initial)+"_optimized_time_ugv-uav.txt", std::ofstream::app);
	file_out_velocity.open (path+"results"+"_stage_"+std::to_string(scenario_number)+"_InitPos_"+std::to_string(num_pos_initial)+"_optimized_velocity_ugv-uav.txt", std::ofstream::app);
	file_out_acceleration.open (path+"results"+"_stage_"+std::to_string(scenario_number)+"_InitPos_"+std::to_string(num_pos_initial)+"_optimized_acceleration_ugv-uav.txt", std::ofstream::app);
	file_out_rotation.open (path+"results"+"_stage_"+std::to_string(scenario_number)+"_InitPos_"+std::to_string(num_pos_initial)+"_optimized_rotation_ugv-uav.txt", std::ofstream::app);
	file_out_kinematic.open (path+"results"+"_stage_"+std::to_string(scenario_number)+"_InitPos_"+std::to_string(num_pos_initial)+"_optimized_kinematic_ugv-uav.txt", std::ofstream::app);

	double sum_dis_pos_ugv_ = 0.0;
	double sum_dis_pos_uav_ = 0.0;
	double new_vel_ugv_ = 0.0;
	double new_vel_uav_ = 0.0;
	double sum_difftime_ugv_ = 0.0;
	double sum_difftime_uav_ = 0.0;
	double min_time_interval = 0.001;
	for (size_t i=0; i < _s -1; ++i){
		if ( i == 0)
			file_out_time << std::setprecision(6) << sum_dis_pos_ugv_ << ";" << vec_time_opt[i] << ";" << sum_dis_pos_uav_ << ";" << vec_time_opt[i] <<"/";
		
		double difftime_ugv_ = vec_time_opt[i+1];
		double difftime_uav_ = vec_time_opt[i+1];
		
		double dist_ugv_ = sqrt((vec_pose_ugv_opt[i].x - vec_pose_ugv_opt[i+1].x)*(vec_pose_ugv_opt[i].x - vec_pose_ugv_opt[i+1].x) +
								(vec_pose_ugv_opt[i].y - vec_pose_ugv_opt[i+1].y)*(vec_pose_ugv_opt[i].y - vec_pose_ugv_opt[i+1].y) +
								(vec_pose_ugv_opt[i].z - vec_pose_ugv_opt[i+1].z)*(vec_pose_ugv_opt[i].z - vec_pose_ugv_opt[i+1].z));
		double dist_uav_ = sqrt((vec_pose_uav_opt[i].x - vec_pose_uav_opt[i+1].x)*(vec_pose_uav_opt[i].x - vec_pose_uav_opt[i+1].x) +
								(vec_pose_uav_opt[i].y - vec_pose_uav_opt[i+1].y)*(vec_pose_uav_opt[i].y - vec_pose_uav_opt[i+1].y) +
								(vec_pose_uav_opt[i].z - vec_pose_uav_opt[i+1].z)*(vec_pose_uav_opt[i].z - vec_pose_uav_opt[i+1].z));
		
		vec_dist_ugv_opt.push_back(dist_ugv_);
		vec_dist_uav_opt.push_back(dist_uav_);
		sum_dis_pos_ugv_ = dist_ugv_ + sum_dis_pos_ugv_;
		sum_dis_pos_uav_ = dist_uav_ + sum_dis_pos_uav_;
		
		// Condition for time UGV	
			sum_difftime_ugv_ = sum_difftime_ugv_ + difftime_ugv_;
		// Condition for time UAV	
			sum_difftime_uav_ = sum_difftime_uav_ + difftime_uav_;

		// Condition for vel UGV	
		if (vec_time_opt[i+1] < min_time_interval)
			new_vel_ugv_  = 0.0;
		else
			new_vel_ugv_ = dist_ugv_ / difftime_ugv_;
		// Condition for vel UAV	
		if (vec_time_opt[i+1] < min_time_interval)
			new_vel_uav_ = 0.0;
		else	
			new_vel_uav_ = dist_uav_ / difftime_uav_;
		
		vec_vel_ugv_opt.push_back(new_vel_ugv_);
		vec_vel_uav_opt.push_back(new_vel_uav_);

		file_out_time << std::setprecision(6) << sum_dis_pos_ugv_ << ";" << sum_difftime_ugv_ << ";" << sum_dis_pos_uav_ << ";" << sum_difftime_uav_ << "/";
		file_out_velocity << std::setprecision(6) << sum_dis_pos_ugv_ << ";" << new_vel_ugv_ << ";" << sum_dis_pos_uav_ << ";" << new_vel_uav_ << "/";

		geometry_msgs::Vector3 rot_ugv_ = getEulerAngles(vec_opt_rot_ugv[i].x,vec_opt_rot_ugv[i].y,vec_opt_rot_ugv[i].z,vec_opt_rot_ugv[i].w);
		geometry_msgs::Vector3 rot_uav_ = getEulerAngles(vec_opt_rot_uav[i].x,vec_opt_rot_uav[i].y,vec_opt_rot_uav[i].z,vec_opt_rot_uav[i].w);

		file_out_rotation  << std::setprecision(6) << rot_ugv_.x << ";" << rot_ugv_.y << ";" << rot_ugv_.z << ";" 
									    		   << rot_uav_.x << ";" << rot_uav_.y << ";" << rot_uav_.z << "/";

	}
	double sum_dis_ugv_ = 0.0;
	double sum_dis_uav_ = 0.0;
	for (size_t i = 0; i < _s-2; i++)
	{
		double difftime1_ugv_ = vec_time_opt[i+1];
		double difftime2_ugv_ = vec_time_opt[i+2];
		double difftime1_uav_ = vec_time_opt[i+1];
		double difftime2_uav_ = vec_time_opt[i+2];

		double distance1_ugv_ = sqrt((vec_pose_ugv_opt[i+1].x - vec_pose_ugv_opt[i].x)*(vec_pose_ugv_opt[i+1].x - vec_pose_ugv_opt[i].x) +
									 (vec_pose_ugv_opt[i+1].y - vec_pose_ugv_opt[i].y)*(vec_pose_ugv_opt[i+1].y - vec_pose_ugv_opt[i].y) +
									 (vec_pose_ugv_opt[i+1].z - vec_pose_ugv_opt[i].z)*(vec_pose_ugv_opt[i+1].z - vec_pose_ugv_opt[i].z));
		double distance2_ugv_ = sqrt((vec_pose_ugv_opt[i+2].x - vec_pose_ugv_opt[i+1].x)*(vec_pose_ugv_opt[i+2].x - vec_pose_ugv_opt[i+1].x) +
									 (vec_pose_ugv_opt[i+2].y - vec_pose_ugv_opt[i+1].y)*(vec_pose_ugv_opt[i+2].y - vec_pose_ugv_opt[i+1].y) +
									 (vec_pose_ugv_opt[i+2].z - vec_pose_ugv_opt[i+1].z)*(vec_pose_ugv_opt[i+2].z - vec_pose_ugv_opt[i+1].z));

		double distance1_uav_ = sqrt((vec_pose_uav_opt[i+1].x - vec_pose_uav_opt[i].x)*(vec_pose_uav_opt[i+1].x - vec_pose_uav_opt[i].x) +
									 (vec_pose_uav_opt[i+1].y - vec_pose_uav_opt[i].y)*(vec_pose_uav_opt[i+1].y - vec_pose_uav_opt[i].y) +
									 (vec_pose_uav_opt[i+1].z - vec_pose_uav_opt[i].z)*(vec_pose_uav_opt[i+1].z - vec_pose_uav_opt[i].z));
		double distance2_uav_ = sqrt((vec_pose_uav_opt[i+2].x - vec_pose_uav_opt[i+1].x)*(vec_pose_uav_opt[i+2].x - vec_pose_uav_opt[i+1].x) +
									 (vec_pose_uav_opt[i+2].y - vec_pose_uav_opt[i+1].y)*(vec_pose_uav_opt[i+2].y - vec_pose_uav_opt[i+1].y) +
									 (vec_pose_uav_opt[i+2].z - vec_pose_uav_opt[i+1].z)*(vec_pose_uav_opt[i+2].z - vec_pose_uav_opt[i+1].z));								 	

		
		if (i==0){
			sum_dis_ugv_ = distance1_ugv_;
			sum_dis_uav_ = distance1_uav_;
		}
		sum_dis_ugv_ = sum_dis_ugv_ + distance2_ugv_;
		sum_dis_uav_ = sum_dis_uav_ + distance2_uav_;
		
		double sumTime_ugv_ = difftime1_ugv_ + difftime2_ugv_;
		double velocity1_ugv_, velocity2_ugv_;
		if (difftime1_ugv_ < min_time_interval)
			velocity1_ugv_ = 0.0;
		else
			velocity1_ugv_ = distance1_ugv_ / difftime1_ugv_;
		if (difftime2_ugv_ < min_time_interval)
			velocity2_ugv_ = 0.0;
		else	
			velocity2_ugv_ = distance2_ugv_ / difftime2_ugv_;
		double acceleration_ugv_;
		if (sumTime_ugv_ < min_time_interval)
			acceleration_ugv_ = 0.0;
		else
			acceleration_ugv_ = (velocity2_ugv_- velocity1_ugv_)/sumTime_ugv_;
		vec_acc_ugv_opt.push_back(acceleration_ugv_);

		double sumTime_uav_ = difftime1_uav_ + difftime2_uav_;
		double velocity1_uav_, velocity2_uav_;
		if (difftime1_uav_ < min_time_interval)
			velocity1_uav_ = 0.0;
		else
			velocity1_uav_ = distance1_uav_ / difftime1_uav_;
		if (difftime2_uav_ < min_time_interval)
			velocity2_uav_ = 0.0;
		else	
			velocity2_uav_ = distance2_uav_ / difftime2_uav_;
		double acceleration_uav_;
		if (sumTime_uav_ < min_time_interval)
			acceleration_uav_;
		else
			acceleration_uav_ = (velocity2_uav_- velocity1_uav_)/sumTime_uav_;
		vec_acc_uav_opt.push_back(acceleration_uav_);

		file_out_acceleration << std::setprecision(6) << sum_dis_ugv_ << ";" << acceleration_ugv_ << ";" << sum_dis_uav_ << ";" << acceleration_uav_ << "/";
	}

	for(size_t i=0 ; i< v_angles_kinematic_ugv_.size() ; i++){
		file_out_kinematic << i << ";" << v_angles_kinematic_ugv_[i] << ";" << v_angles_kinematic_uav_[i] << "/";
	}

	file_out_time << std::endl;
	file_out_time.close();
	file_out_velocity << std::endl;
	file_out_velocity.close();
	file_out_acceleration << std::endl;
	file_out_acceleration.close();
	file_out_rotation << std::endl;
	file_out_rotation.close();
	file_out_kinematic << std::endl;
	file_out_kinematic.close();
}

inline void DataManagement::getDataForOptimizerAnalysis(pcl::KdTreeFLANN <pcl::PointXYZ> kdt_, pcl::KdTreeFLANN <pcl::PointXYZ> kdt_all_, 
					pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_points_, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_points_all_, 
					double opt_compute_time_ , std::string mode_, int &n_coll_init_path_, int &n_coll_opt_traj_)
{
	//mode = 1 , UGV  - mode = 2 , UAV
	std::vector<geometry_msgs::Point> v_points_catenary_opt_,v_points_catenary_init_;
	std::vector<double> vec_time_init_, vec_time_opt_; 
	std::vector<double> vec_dist_init_, vec_dist_opt_;
	std::vector<double> vec_vel_opt_;
	std::vector<double> vec_acc_opt_;
	std::vector<geometry_msgs::Vector3> vec_pose_init_;
	std::vector<geometry_msgs::Vector3> vec_pose_opt_;
	bool use_oct_full = true;
	mode = mode_;
	vec_time_init_ =  vec_time_init; 
	vec_time_opt_ = vec_time_opt;
	
	if (mode_ == "UGV"){
		// vec_time_init_ =  vec_time_init_ugv; 
		vec_pose_init_ =  vec_pose_init_ugv; 
		vec_dist_init_ = vec_dist_init_ugv;
		// vec_time_opt_ = vec_time_ugv_opt;
		vec_vel_opt_ = vec_vel_ugv_opt;
		vec_acc_opt_ = vec_acc_ugv_opt;
		vec_pose_opt_ = vec_pose_ugv_opt;
		vec_dist_opt_ = vec_dist_ugv_opt;
		use_oct_full = false;
		num_points_coll_cat = num_cat_coll = 0; 
		num_coll_ugv_traj = 0;
		pos_coll_cat = "";
	}
	else if (mode_ == "UAV"){
		// vec_time_init_ =  vec_time_init_uav; 
		vec_pose_init_ = vec_pose_init_uav;
		vec_dist_init_ = vec_dist_init_uav;
		// vec_time_opt_ = vec_time_uav_opt;
		vec_vel_opt_ = vec_vel_uav_opt;
		vec_acc_opt_ = vec_acc_uav_opt;
		vec_pose_opt_ = vec_pose_uav_opt;
		vec_dist_opt_ = vec_dist_uav_opt;
		use_oct_full = true;
		num_coll_uav_traj = 0;
	}

	// Writing general data for initial analysis
	double init_compute_time_, init_traj_distance_, init_traj_time_, init_traj_vel_, init_traj_vel_max_, init_traj_vel_mean_, init_traj_acc_,init_traj_acc_max_,init_traj_acc_mean_;
	init_compute_time_ = init_traj_distance_ = init_traj_time_ = init_traj_vel_ = init_traj_vel_max_ = init_traj_vel_mean_ = init_traj_acc_ = init_traj_acc_max_ = init_traj_acc_mean_=0.0;
	int count_dist_init_ = 0;
		

	//Length and Time Trajectory Initial
	for(size_t i = 0 ; i < vec_time_init_.size()-1 ; i++){
		init_traj_time_ = vec_time_init_[i+1] + init_traj_time_;
		init_traj_distance_ = vec_dist_init_[i] + init_traj_distance_;

		double value_dist_init_; 
		if ((vec_dist_init_[i+1]-vec_dist_init_[i]) < 0.0001)
			value_dist_init_ = 0.0;
		else{	
			value_dist_init_ = (vec_dist_init_[i] / vec_time_init_[i+1]);
			count_dist_init_++;
		}
		init_traj_vel_ = value_dist_init_ + init_traj_vel_; 
		
		if (init_traj_vel_max_ < value_dist_init_)	
			init_traj_vel_max_ = value_dist_init_;
	}
	init_traj_vel_mean_ = init_traj_vel_ / ((double)count_dist_init_+ 1.0);

	int count_acc_init_ = 0;
	//Acceleration Trajectory Initial
	for(size_t i = 0 ; i < vec_time_init_.size()-2 ; i++){
		double value_dist_init_1_, value_dist_init_2_;
		if( (vec_time_init_[i+1] - vec_time_init_[i]) < 0.0001 )
			value_dist_init_1_ = 0.0;
		else
			value_dist_init_1_ = vec_dist_init_[i] / vec_time_init_[i+1];
		
		if( (vec_time_init_[i+2] - vec_time_init_[i+1]) < 0.0001 )
			value_dist_init_2_ = 0.0;
		else
			value_dist_init_2_ = vec_dist_init_[i+1]/vec_time_init_[i+2];
		
		double value_acc_init_;
		if( (vec_time_init_[i+2] - vec_time_init_[i+1]) < 0.0001 )
			value_acc_init_ = 0.0 ; 
		else{	
			value_acc_init_ = ( ((value_dist_init_1_) - (value_dist_init_2_)) / (vec_time_init_[i+1] + vec_time_init_[i+2]) ) ;
			count_acc_init_++;
		}

		init_traj_acc_ = value_acc_init_ + init_traj_acc_; 

		
		if (init_traj_acc_max_ < ( value_acc_init_ ) )
			init_traj_acc_max_ = ( value_acc_init_ );
	}
	init_traj_acc_mean_ = init_traj_acc_ / ((double)count_acc_init_+1.0);

	//Distance Obstacles Initial
	double distance_obs_init_ , distance_obs_init_min_, distance_obs_init_mean_;
	distance_obs_init_ = distance_obs_init_mean_ = 0.0;
	distance_obs_init_min_ = 1000.0;
	for(size_t i = 0 ; i < vec_pose_init_.size(); i ++){
		Eigen::Vector3d p_init_ = Eigen::Vector3d(vec_pose_init_[i].x,vec_pose_init_[i].y,vec_pose_init_[i].z);
		Eigen::Vector3d nearest_obs_p_ = nn_.nearestObstacleVertex(kdt_, p_init_, obstacles_points_);
		distance_obs_init_ = (p_init_- nearest_obs_p_).norm() + distance_obs_init_;
		if(distance_obs_init_min_ > (p_init_- nearest_obs_p_).norm() )
			distance_obs_init_min_ = (p_init_- nearest_obs_p_).norm();
	}
	distance_obs_init_mean_ = distance_obs_init_ / (double)vec_pose_init_.size();

	//Distance Catenary Obstacles Initial
	double distance_obs_cat_init_ , distance_obs_cat_init_min_, distance_obs_cat_init_mean_;
	distance_obs_cat_init_ = distance_obs_cat_init_mean_ = 0.0;
	distance_obs_cat_init_min_ = 1000.0;
	int count_cat_p_init_ = 0;
	int count_coll_between_init_ = 0;
	std::string pos_coll_between_init_;
	
	bisectionCatenary bc1;
	bisectionCatenary bc2;

	// Initial Catenary analisys 
	for(size_t i = 0 ; i < vec_pose_init_.size(); i ++){
		// CatenarySolver cS_;
		v_points_catenary_init_.clear();
		geometry_msgs::Vector3 p_reel_ugv;
		p_reel_ugv = getReelPos(vec_pose_init_ugv[i].x,vec_pose_init_ugv[i].y,vec_pose_init_ugv[i].z,vec_init_rot_ugv[i].x,vec_init_rot_ugv[i].y,vec_init_rot_ugv[i].z,vec_init_rot_ugv[i].w, pos_reel_ugv);
		// cS_.setMaxNumIterations(100);
		// cS_.solve(p_reel_ugv.x, p_reel_ugv.y, p_reel_ugv.z, vec_pose_init_uav[i].x(), vec_pose_init_uav[i].y(), vec_pose_init_uav[i].z(), vec_len_cat_init[i], v_points_catenary_init_);
		bc1.configBisection(vec_len_cat_init[i], p_reel_ugv.x, p_reel_ugv.y, p_reel_ugv.z, 
							vec_pose_init_uav[i].x, vec_pose_init_uav[i].y, vec_pose_init_uav[i].z, false);
		if (bc1.L_minor_than_D == true)
			printf("Before optimization , position[%lu/%lu]: L < D \n",i,vec_pose_opt_.size());		
		bc1.getPointCatenary3D(v_points_catenary_init_);
		
		for (size_t j= 0 ; j < v_points_catenary_init_.size() ; j++){
			count_cat_p_init_++;
			Eigen::Vector3d p_cat_init_ = Eigen::Vector3d(v_points_catenary_init_[j].x, v_points_catenary_init_[j].y, v_points_catenary_init_[j].z);
			Eigen::Vector3d nearest_obs_p = nn_.nearestObstacleVertex(kdt_all_, p_cat_init_, obstacles_points_all_);
			distance_obs_cat_init_ = (p_cat_init_- nearest_obs_p).norm() + distance_obs_cat_init_;
			if(distance_obs_cat_init_min_ > (p_cat_init_- nearest_obs_p).norm() )
				distance_obs_cat_init_min_ = (p_cat_init_- nearest_obs_p).norm();
		}
		// if ( (i < vec_pose_init_.size()-1) && (isObstacleBetweenTwoPoints(vec_pose_init_[i], vec_pose_init_[i+1], use_oct_full)) ){
		if ( (i < vec_pose_init_.size()-1) ){
			count_coll_between_init_++;
			pos_coll_between_init_ = pos_coll_between_init_+"["+std::to_string(i)+"-"+std::to_string(i+1)+"]";
		}
	}
	distance_obs_cat_init_mean_ = distance_obs_cat_init_/ (double)count_cat_p_init_;

	// Writing general data for optimized analysis
	double opt_traj_distance_, opt_traj_time_, opt_traj_vel_, opt_traj_vel_max_, opt_traj_vel_mean_, opt_traj_acc_, opt_traj_acc_max_, opt_traj_acc_mean_;
	opt_traj_distance_ = opt_traj_time_ = opt_traj_vel_ = opt_traj_vel_max_ = opt_traj_vel_mean_ = opt_traj_acc_ = opt_traj_acc_max_ = opt_traj_acc_mean_ = 0.0;
	int count_dist_opt_ = 0;

	//Length, Time, Velocity Trajectory Optimized
	for (size_t i = 0; i < vec_time_opt_.size() -1 ; i++){
		opt_traj_time_ = vec_time_opt_[i+1] + opt_traj_time_;
		opt_traj_distance_ = vec_dist_opt_[i] + opt_traj_distance_;

		if (vec_vel_opt_[i+1])
		opt_traj_vel_ = vec_vel_opt_[i] + opt_traj_vel_;

		if (opt_traj_vel_max_ < vec_vel_opt_[i])
			opt_traj_vel_max_ = vec_vel_opt_[i];
	}
	opt_traj_vel_mean_ = opt_traj_vel_ / ((double)vec_time_opt_.size()-1.0);

	//Acceleration Trajectory Optimized
	for (size_t i = 0; i < vec_acc_opt_.size() ; i++){
		opt_traj_acc_ = vec_acc_opt_[i] + opt_traj_acc_;
		if (fabs(opt_traj_acc_max_) < fabs(vec_acc_opt_[i]))
			opt_traj_acc_max_ = vec_acc_opt_[i];
	}
	opt_traj_acc_mean_ = opt_traj_acc_ / (double)vec_acc_opt_.size();

	//Distance Point Obstacles Optimized
	double distance_obs_opt_ , distance_obs_opt_min_, distance_obs_opt_mean_;
	distance_obs_opt_ = distance_obs_opt_mean_ = 0.0;
	distance_obs_opt_min_ = 1000.0;
	for(size_t i = 0 ; i < vec_pose_opt_.size(); i ++){
		Eigen::Vector3d p_opt_ = Eigen::Vector3d(vec_pose_opt_[i].x, vec_pose_opt_[i].y, vec_pose_opt_[i].z);
		Eigen::Vector3d nearest_obs_p_ = nn_.nearestObstacleVertex(kdt_, p_opt_ , obstacles_points_);
		distance_obs_opt_ = (p_opt_- nearest_obs_p_).norm() + distance_obs_opt_;
		if(distance_obs_opt_min_ > (p_opt_- nearest_obs_p_).norm() )
			distance_obs_opt_min_ = (p_opt_- nearest_obs_p_).norm();
	}
	distance_obs_opt_mean_ = distance_obs_opt_ / (double)vec_pose_opt_.size();

	//Distance Catenary Obstacles Optimized
	double distance_obs_cat_opt_ , distance_obs_cat_opt_min_, distance_obs_cat_opt_mean_;
	distance_obs_cat_opt_ = distance_obs_cat_opt_mean_ = 0.0;
	distance_obs_cat_opt_min_ = 1000.0;
	int count_cat_p_opt_ = 0;
	int count_coll_between_opt_ = 0;
	std::string pos_coll_between_opt_ = "";

	// Optimized Catenary analisys 
	for(size_t i = 0 ; i < vec_pose_opt_.size(); i ++){
		// CatenarySolver cS_;
		v_points_catenary_opt_.clear();
		geometry_msgs::Vector3 p_reel_ugv;
		std::vector<double> dist_obst_cat; 
		std::vector<int> cat_between_obs, pos_cat_in_coll;
		dist_obst_cat.clear(); pos_cat_in_coll.clear(); cat_between_obs.clear(); 

		int first_coll_, last_coll_;
		first_coll_ = last_coll_ = 0;
		p_reel_ugv = getReelPos(vec_pose_ugv_opt[i].x,vec_pose_ugv_opt[i].y,vec_pose_ugv_opt[i].z,vec_opt_rot_ugv[i].x,vec_opt_rot_ugv[i].y,vec_opt_rot_ugv[i].z,vec_opt_rot_ugv[i].w, pos_reel_ugv);
		// cS_.setMaxNumIterations(100);
		// cS_.solve(p_reel_ugv.x, p_reel_ugv.y, p_reel_ugv.z, vec_pose_uav_opt[i].x(), vec_pose_uav_opt[i].y(), vec_pose_uav_opt[i].z(), vec_len_cat_opt[i], v_points_catenary_opt_);
		bc2.readDataForCollisionAnalisys(g_3D_, bound_cat_obs, octree_full, kdt_, obstacles_points_);
		bc2.configBisection(vec_len_cat_opt[i], p_reel_ugv.x, p_reel_ugv.y, p_reel_ugv.z,vec_pose_uav_opt[i].x,vec_pose_uav_opt[i].y,vec_pose_uav_opt[i].z,true);
		if (bc2.L_minor_than_D == true)
			printf("After optimization , position[%lu/%lu]: L < D \n",i,vec_pose_opt_.size());		
		bc2.getPointCatenary3D(v_points_catenary_opt_);
		bc2.getStatusCollisionCat(dist_obst_cat, pos_cat_in_coll, cat_between_obs, first_coll_, last_coll_);
		
		num_points_coll_cat = 0;
		double min_val_proximity_ = 0.015;
		for (size_t j= 0 ; j < dist_obst_cat.size() ; j++){
			// printf("dist_obst_cat[%lu]=%f \n",j, dist_obst_cat[j]);
			count_cat_p_opt_++;
			double d_diff = dist_obst_cat[j];
			if (d_diff < min_val_proximity_)
				d_diff = min_val_proximity_ ;
			distance_obs_cat_opt_ = d_diff + distance_obs_cat_opt_;
			if(distance_obs_cat_opt_min_ > d_diff )
				distance_obs_cat_opt_min_ = d_diff;
			if( d_diff < bound_cat_obs || (j >= first_coll_ && j <= last_coll_) && j!= 0){
				num_points_coll_cat++; 
				// if(mode_ == "UAV")
					// printf("Catenary= %lu , d_diff=[%.2f/%.2f] , position_Catenary=[%lu-%lu]/[%lu] vec_len_cat_opt=[%f] first_coll=[%i] last_coll=[%i]\n",i,d_diff,bound_cat_obs ,j,j-1,v_points_catenary_opt_.size(),vec_len_cat_opt[i], first_coll_, last_coll_);
			}
		}
		// Next to count how many catenary are in collision in the trajectory
		if (num_points_coll_cat > 0 && first_coll_ != 0 && last_coll_ != 0 && mode_ == "UAV"){
			num_cat_coll++;	
			pos_coll_cat = pos_coll_cat+std::to_string(i)+"-";
		} 
		// Next to count the number of consecutive states in collision depending on mode (UAV or UGV)
		// if ( (i < vec_pose_opt_.size()-1) && (isObstacleBetweenTwoPoints(vec_pose_opt_[i], vec_pose_opt_[i+1], use_oct_full)) ){
		if ( (i < vec_pose_opt_.size()-1)  ){
			count_coll_between_opt_++;
			pos_coll_between_opt_ = pos_coll_between_opt_+"["+std::to_string(i)+"-"+std::to_string(i+1)+"]";
			if (mode_ == "UGV")
				num_coll_ugv_traj = count_coll_between_opt_;
			if (mode_ == "UAV"){ 
				num_coll_uav_traj = count_coll_between_opt_;
				// printf("Collision en nodo num=[%lu]\n",i);
			}
		}
	}
	if (num_cat_coll == 0 && mode_ == "UAV")
		pos_coll_cat = "-";

	distance_obs_cat_opt_mean_ = distance_obs_cat_opt_/ (double)count_cat_p_opt_;

	

	std::string name_output_file = output_file +"_"+ mode_ +".txt";

	ofs.open(name_output_file.c_str(), std::ofstream::app);

	if (ofs.is_open()) {
		std::cout << "Saving data in output file for "<< mode_ << " : " << name_output_file << std::endl;
		ofs << opt_compute_time_ << ";" 
		    << init_traj_distance_ << ";" 
		    << opt_traj_distance_ << ";"
			<< init_traj_time_ << ";" 
			<< opt_traj_time_ << ";" 
			<< distance_obs_init_mean_ << ";"
			<< distance_obs_init_min_ << ";"
			<< distance_obs_opt_mean_ << ";" 
			<< distance_obs_opt_min_ << ";" 
			<< distance_obs_cat_init_mean_ << ";" 
			<< distance_obs_cat_init_min_ << ";" 
			<< distance_obs_cat_opt_mean_ << ";" 
			<< distance_obs_cat_opt_min_ << ";"
			<< init_traj_vel_mean_ << ";" 
			<< init_traj_vel_max_ << ";"
			<< opt_traj_vel_mean_ << ";"
			<< opt_traj_vel_max_ << ";"
			<< init_traj_acc_mean_ << ";"
			<< init_traj_acc_max_ << ";" 
			<< opt_traj_acc_mean_ << ";"
			<< opt_traj_acc_max_ << ";"
			<< count_coll_between_init_ << ";"
			<< pos_coll_between_init_<< ";"
			<< count_coll_between_opt_ << ";"
			<< pos_coll_between_opt_ 
			<<std::endl;
	} 
	else {
		std::cout << "Couldn't be open the output data file for " << mode_ << std::endl;
	}
	n_coll_init_path_ = count_coll_between_init_;
	n_coll_opt_traj_ =  count_coll_between_opt_;
	ofs.close();
}

inline geometry_msgs::Vector3 DataManagement::getReelPos(const float px_, const float py_, const float pz_, const float qx_, const float qy_, const float qz_,
					const float qw_, geometry_msgs::Vector3 p_reel_)
{
	geometry_msgs::Vector3 ret;

	double roll_, pitch_, yaw_;
	tf::Quaternion q_(qx_,qy_,qz_,qw_);
	tf::Matrix3x3 M_(q_);	
	M_.getRPY(roll_, pitch_, yaw_);

	double lengt_vec =  sqrt(p_reel_.x*p_reel_.x + p_reel_.y*p_reel_.y);
	ret.x = px_ + lengt_vec *cos(yaw_); 
	ret.y = py_ + lengt_vec *sin(yaw_);
	ret.z = pz_ + p_reel_.z ;

	return ret;
}

inline geometry_msgs::Vector3 DataManagement::getEulerAngles(const float qx_, const float qy_, const float qz_, const float qw_)
{
	geometry_msgs::Vector3 ret;

	double roll_, pitch_, yaw_;
	tf::Quaternion q_(qx_,qy_,qz_,qw_);
	tf::Matrix3x3 M_(q_);	
	M_.getRPY(roll_, pitch_, yaw_);

	ret.x = roll_; 
	ret.y = pitch_;
	ret.z = yaw_;

	return ret;
}

inline bool DataManagement::isObstacleBetweenTwoPoints(geometry_msgs::Vector3 pose_opt_1, geometry_msgs::Vector3 pose_opt_2, bool oct_full_)
{
	octomap::point3d r_;

	octomap::point3d s_(pose_opt_1.x , pose_opt_1.y , pose_opt_1.z +0.3); //start for rayCast
	octomap::point3d d_(pose_opt_2.x-s_.x() , pose_opt_2.y-s_.y() , pose_opt_2.z-s_.z() ); //direction for rayCast

	if(d_.x()== 0 && d_.y()== 0 && d_.z()==0)
		return false;
    bool r_cast_coll = false; 
    
	if(oct_full_)
		r_cast_coll = octree_full->castRay(s_, d_, r_);
	else
		r_cast_coll = octree_ugv->castRay(s_, d_, r_);

	double d_12 = sqrt( (pose_opt_2.x-s_.x())*(pose_opt_2.x-s_.x())+
					   	(pose_opt_2.y-s_.y())*(pose_opt_2.y-s_.y())+ 
					   	(pose_opt_2.z-s_.z())*(pose_opt_2.z-s_.z()) );
	
	double d_obs = sqrt ((s_.x()-r_.x())*(s_.x()-r_.x()) + (s_.y()-r_.y())*(s_.y()-r_.y()) + (s_.z()-r_.z())*(s_.z()-r_.z()) );

	if (r_cast_coll && d_obs <= d_12){
		// printf(" r_=[%f %f %f] s_=[%f %f %f] d_=[%f %f %f] dist12_=[%f] distObs_=[%f] r_cast_coll=%s\n",r_.x(),r_.y(),r_.z(),
		// 																				s_.x(),s_.y(),s_.z(),
		// 																				d_.x(),d_.y(),d_.z(),
		// 																				d_12, d_obs, r_cast_coll?"true":"false");
		return true;
	}
	else
		return false;		
}

inline void DataManagement::feasibilityAnalisysTrajectory(float init_cost, float final_cost, float succes_steps, float unsuccess_step, float time_opt, int &n_coll_opt_cat_)
{
	std::string name_file = path+"results"+"_stage_"+std::to_string(scenario_number)+"_InitPos_"+std::to_string(num_pos_initial)+"_"+
							"feasibility_trajectory.txt";

	std::ifstream ifile;
   	ifile.open(name_file);
   	if(ifile) {
      	std::cout << name_file <<" : File exists !!!!!!!!!! " << std::endl;
   	} else {
	  feasibility.open(name_file.c_str(), std::ofstream::app);
	  feasibility << "Feasible/Coll_ugv/Coll_uav/Cat_coll/PosCollCat/C_cost/I_cost/F_cost/T_step/S_step/UnS_steps/time_opt"<<std::endl;
	  feasibility.close();
      std::cout << name_file <<" : File doesn't exist !!!!!!!!!! " << std::endl;
   	}
	
	feasibility.open(name_file.c_str(), std::ofstream::app);

	n_coll_opt_cat_ = num_cat_coll;

	int feasible = 0;
	if (num_coll_ugv_traj == 0 && num_coll_uav_traj == 0 && num_cat_coll == 0)
		feasible = 1;
	
	if (feasibility.is_open()) {
		feasibility << feasible << ";"
					<< num_coll_ugv_traj << ";" 
		    		<< num_coll_uav_traj << ";" 
		    		<< num_cat_coll << ";["
					<< pos_coll_cat << "/"
					<< vec_pose_uav_opt.size() <<"];"
					<< init_cost - final_cost << ";"
					<< init_cost << ";"
					<< final_cost << ";"
					<< succes_steps + unsuccess_step << ";"
					<< succes_steps << ";"
					<< unsuccess_step << ";" 
					<< time_opt << ";"
					<<std::endl;
	} 
	else 
		std::cout << "Couldn't be open result_feasibility_analisys_for_trajectory.txt "<< std::endl;
	feasibility.close();
}


#endif