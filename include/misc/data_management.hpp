#ifndef DATA_MANAGEMENT_HPP
#define DATA_MANAGEMENT_HPP

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include "near_neighbor.hpp"
#include "catenary_solver_ceres.hpp"

#include <ros/ros.h>
#include <geometry_msgs/Point.h> 

#include "Eigen/Core"

#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <octomap_msgs/Octomap.h> 
#include <octomap/OcTree.h>


class DataManagement
{
	public:
		DataManagement();
		// ~DataManagement(){};
		virtual void initDataManagement(std::string path_, std::string name_output_file_, int scenario_number_, int num_pos_initial_, int num_goal_, double initial_velocity_ugv_,
								double initial_velocity_uav_, double initial_acceleration_ugv_, double initial_acceleration_uav_, geometry_msgs::Point pos_reel_ugv_,
								std::vector<Eigen::Vector3d> vec_pose_init_ugv_, std::vector<Eigen::Vector3d> vec_pose_init_uav_,	std::vector<float> vec_len_cat_init_,
								std::vector<geometry_msgs::Quaternion> vec_rot_ugv_, octomap::OcTree* octotree_);
		virtual void writeTemporalDataBeforeOptimization(std::vector<double> vec_dist_init_ugv_, std::vector<double> vec_dist_init_uav_, std::vector<double> vec_time_init_ugv_, std::vector<double> vec_time_init_uav_);
		void writeTemporalDataAfterOptimization(auto _s, std::vector<Eigen::Vector3d> vec_pose_ugv_opt_, std::vector<Eigen::Vector3d> vec_pose_uav_opt_, 
														std::vector<double> vec_time_ugv_opt_, std::vector<double> vec_time_uav_opt_, std::vector<float> vec_len_cat_opt_);
		virtual void getDataForOptimizerAnalysis(pcl::KdTreeFLANN <pcl::PointXYZ> kdt_uav, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_points_uav, double opt_compute_time_);
 		virtual geometry_msgs::Point getReelPos(const float px_, const float py_, const float pz_,const float qx_, const float qy_, const float qz_, const float qw_, geometry_msgs::Point p_reel_);
		virtual bool isObstacleBetweenTwoPoints(Eigen::Vector3d pose_uav_opt_1, Eigen::Vector3d pose_uav_opt_2);

		std::string path;
		std::string output_file, name_output_file;
		int scenario_number, num_pos_initial, num_goal;
		std::ofstream ofs;
		std::ofstream file_in_time ,file_in_velocity, file_in_acceleration;
		std::ofstream file_out_time, file_out_velocity, file_out_acceleration;

		std::vector<Eigen::Vector3d> vec_pose_ugv_opt, vec_pose_uav_opt;
		std::vector<double> vec_time_ugv_opt, vec_time_uav_opt;
		std::vector<double> vec_dist_ugv_opt, vec_dist_uav_opt, vec_vel_ugv_opt, vec_vel_uav_opt, vec_acc_ugv_opt, vec_acc_uav_opt;
		std::vector<Eigen::Vector3d> vec_pose_init_ugv, vec_pose_init_uav;

		double initial_velocity_ugv, initial_velocity_uav, initial_acceleration_ugv, initial_acceleration_uav;

		std::vector<double> vec_dist_init_ugv, vec_dist_init_uav;
		std::vector<double> vec_time_init_ugv, vec_time_init_uav;
		std::vector<float> vec_len_cat_init, vec_len_cat_opt;
		std::vector<geometry_msgs::Quaternion> vec_rot_ugv;

	    NearNeighbor nn_uav;

		geometry_msgs::Point ugv_pos_catenary;
		geometry_msgs::Point pos_reel_ugv;

		octomap::OcTree* octotree;

	protected:

	private:

};

inline DataManagement::DataManagement(){}

inline void DataManagement::initDataManagement(std::string path_, std::string name_output_file_, int scenario_number_, int num_pos_initial_, int num_goal_, double initial_velocity_ugv_,
										double initial_velocity_uav_, double initial_acceleration_ugv_, double initial_acceleration_uav_, geometry_msgs::Point pos_reel_ugv_,
										std::vector<Eigen::Vector3d> vec_pose_init_ugv_, std::vector<Eigen::Vector3d> vec_pose_init_uav_, std::vector<float> vec_len_cat_init_,
										std::vector<geometry_msgs::Quaternion> vec_rot_ugv_, octomap::OcTree* octotree_)
{
	path = path_;
	name_output_file = name_output_file_;
	scenario_number = scenario_number_; 
	num_pos_initial = num_pos_initial_;
	num_goal = num_goal_;

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
	vec_rot_ugv = vec_rot_ugv_;
	
	pos_reel_ugv = pos_reel_ugv_;

	octotree = octotree_;

	output_file = path+name_output_file+"_stage_"+std::to_string(scenario_number)+"_InitPos_"+std::to_string(num_pos_initial)+"_goal_"+std::to_string(num_goal)+".txt";
	ofs.open(output_file.c_str(), std::ofstream::app);
}

inline void DataManagement::writeTemporalDataBeforeOptimization(std::vector<double> vec_dist_init_ugv_, std::vector<double> vec_dist_init_uav_, std::vector<double> vec_time_init_ugv_, std::vector<double> vec_time_init_uav_)
{
	vec_dist_init_ugv.clear(); 
	vec_dist_init_uav.clear(); 
	vec_time_init_ugv.clear();
	vec_time_init_uav.clear();
	vec_dist_init_ugv = vec_dist_init_ugv_;
	vec_dist_init_uav = vec_dist_init_uav_;
	vec_time_init_ugv = vec_time_init_ugv_;
	vec_time_init_uav = vec_time_init_uav_;

	//! Save temporal state before optimization
	file_in_time.open (path+"initial_time.txt");
	file_in_velocity.open (path+"initial_velocity.txt");
	file_in_acceleration.open (path+"initial_acceleration.txt");
	
	double _sum_dist_ugv = 0.0;
	double _sum_time_ugv = 0.0;
	double _sum_dist_uav = 0.0;
	double _sum_time_uav = 0.0;

	for (size_t i = 0; i < vec_time_init_uav.size() - 1; i++){
		_sum_dist_ugv = _sum_dist_ugv + vec_dist_init_ugv[i];
		_sum_time_ugv = _sum_time_ugv + vec_time_init_ugv[i+1];
		_sum_dist_uav = _sum_dist_uav + vec_dist_init_uav[i];
		_sum_time_uav = _sum_time_uav + vec_time_init_uav[i+1];

		file_in_time << std::setprecision(6) << _sum_dist_ugv << ";" << _sum_time_ugv << ";" << _sum_dist_uav << ";" << _sum_time_uav << std::endl;
		file_in_velocity  << std::setprecision(6) << _sum_dist_ugv << ";" << initial_velocity_ugv << ";" << _sum_dist_uav << ";" << initial_velocity_uav << std::endl;
		if (i > 0)
			file_in_acceleration << std::setprecision(6) << _sum_dist_ugv << ";" << initial_acceleration_ugv << ";" << _sum_dist_uav << ";" << initial_acceleration_uav << std::endl;
	}
	file_in_time.close();	
	file_in_velocity.close();	
	file_in_acceleration.close();
}

inline void DataManagement::writeTemporalDataAfterOptimization(auto _s, std::vector<Eigen::Vector3d> vec_pose_ugv_opt_, std::vector<Eigen::Vector3d> vec_pose_uav_opt_, 
														std::vector<double> vec_time_ugv_opt_, std::vector<double> vec_time_uav_opt_, std::vector<float> vec_len_cat_opt_)
{
	vec_pose_ugv_opt.clear();
	vec_pose_uav_opt.clear();
	vec_time_ugv_opt.clear();
	vec_time_uav_opt.clear();
	vec_len_cat_opt.clear();
	vec_pose_ugv_opt = vec_pose_ugv_opt_;
	vec_pose_uav_opt = vec_pose_uav_opt_;
	vec_time_ugv_opt = vec_time_ugv_opt_;
	vec_time_uav_opt = vec_time_uav_opt_;
	vec_len_cat_opt = vec_len_cat_opt_;
	
	vec_dist_ugv_opt.clear(); 
	vec_dist_uav_opt.clear(); 
	vec_vel_ugv_opt.clear(); 
	vec_vel_uav_opt.clear(); 
	vec_acc_ugv_opt.clear();
	vec_acc_uav_opt.clear();

	//! Save Temporal Data Optimized in File.txt 
	file_out_time.open (path+"optimized_time.txt");
	file_out_velocity.open (path+"optimized_velocity.txt");
	file_out_acceleration.open (path+"optimized_acceleration.txt");
	double sum_dis_pos_ugv_ = 0.0;
	double sum_dis_pos_uav_ = 0.0;
	double new_vel_ugv_ = 0.0;
	double new_vel_uav_ = 0.0;
	double sum_difftime_ugv_ = 0.0;
	double sum_difftime_uav_ = 0.0;
	double min_time_interval = 0.001;
	for (size_t i=0; i < _s -1; ++i){
		if ( i == 0)
			file_out_time << std::setprecision(6) << sum_dis_pos_ugv_ << ";" << vec_time_ugv_opt[i] << ";" << sum_dis_pos_uav_ << ";" << vec_time_uav_opt[i] << std::endl;
		double difftime_ugv_ = vec_time_ugv_opt[i+1];
		double difftime_uav_ = vec_time_uav_opt[i+1];
		double dist_ugv_ = (vec_pose_ugv_opt[i] - vec_pose_ugv_opt[i+1]).norm();
		double dist_uav_ = (vec_pose_uav_opt[i] - vec_pose_uav_opt[i+1]).norm();
		vec_dist_ugv_opt.push_back(dist_ugv_);
		vec_dist_uav_opt.push_back(dist_uav_);
		sum_dis_pos_ugv_ = dist_ugv_ + sum_dis_pos_ugv_;
		sum_dis_pos_uav_ = dist_uav_ + sum_dis_pos_uav_;
		if (difftime_ugv_ < min_time_interval)
			new_vel_ugv_  = 0.0;
		else
			new_vel_ugv_ = dist_ugv_ / difftime_ugv_;
		if (difftime_uav_ < min_time_interval)
			new_vel_uav_ = 0.0;
		else	
			new_vel_uav_ = dist_uav_ / difftime_uav_;
		vec_vel_ugv_opt.push_back(new_vel_ugv_);
		vec_vel_uav_opt.push_back(new_vel_uav_);
		sum_difftime_ugv_ = sum_difftime_ugv_ + difftime_ugv_;
		sum_difftime_uav_ = sum_difftime_uav_ + difftime_uav_;
		file_out_time << std::setprecision(6) << sum_dis_pos_ugv_ << ";" << sum_difftime_ugv_ << ";" << sum_dis_pos_uav_ << ";" << sum_difftime_uav_ << std::endl;
		file_out_velocity << std::setprecision(6) << sum_dis_pos_ugv_ << ";" << new_vel_ugv_ << ";" << sum_dis_pos_uav_ << ";" << new_vel_uav_ << std::endl;
	}
	double sum_dis_ugv_ = 0.0;
	double sum_dis_uav_ = 0.0;
	for (size_t i = 0; i < _s-2; i++)
	{
		double difftime1_ugv_ = vec_time_ugv_opt[i+1];
		double difftime2_ugv_ = vec_time_ugv_opt[i+2];
		double difftime1_uav_ = vec_time_uav_opt[i+1];
		double difftime2_uav_ = vec_time_uav_opt[i+2];

		double distance1_ugv_ = (vec_pose_ugv_opt[i+1] - vec_pose_ugv_opt[i]).norm();	
		double distance2_ugv_ = (vec_pose_ugv_opt[i+2] - vec_pose_ugv_opt[i+1]).norm();
		double distance1_uav_ = (vec_pose_uav_opt[i+1] - vec_pose_uav_opt[i]).norm();	
		double distance2_uav_ = (vec_pose_uav_opt[i+2] - vec_pose_uav_opt[i+1]).norm();
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

		file_out_acceleration << std::setprecision(6) << sum_dis_ugv_ << ";" << acceleration_ugv_ << ";" << sum_dis_uav_ << ";" << acceleration_uav_ << std::endl;
	}
	file_out_time.close();
	file_out_velocity.close();
	file_out_acceleration.close();
}

inline void DataManagement::getDataForOptimizerAnalysis(pcl::KdTreeFLANN <pcl::PointXYZ> kdt_uav, pcl::PointCloud <pcl::PointXYZ>::Ptr obstacles_points_uav, double opt_compute_time_)
{
	// bisectionCatenary bsCat;
	std::vector<geometry_msgs::Point> v_points_catenary_opt_,v_points_catenary_init_;

	// Writing general data for initial analysis
	double init_compute_time_, init_traj_distance_, init_traj_time_, init_traj_vel_, init_traj_vel_max_, init_traj_vel_mean_, init_traj_acc_,init_traj_acc_max_,init_traj_acc_mean_;
	init_compute_time_ = init_traj_distance_ = init_traj_time_ = init_traj_vel_ = init_traj_vel_max_ = init_traj_vel_mean_ = init_traj_acc_ = init_traj_acc_max_ = init_traj_acc_mean_=0.0;

	//Length and Time Trajectory Initial
	for(size_t i = 0 ; i < vec_time_init_uav.size()-1 ; i++){
		init_traj_time_ = vec_time_init_uav[i+1] + init_traj_time_;
		init_traj_distance_ = vec_dist_init_uav[i] + init_traj_distance_;
		init_traj_vel_ = (vec_dist_init_uav[i] / vec_time_init_uav[i+1]) + init_traj_vel_; 
		if (init_traj_vel_max_ < (vec_dist_init_uav[i] / vec_time_init_uav[i+1]))
			init_traj_vel_max_ = (vec_dist_init_uav[i] / vec_time_init_uav[i+1]);
	}
	init_traj_vel_mean_ = init_traj_vel_ / ((double)vec_time_init_uav.size()-1.0);

	//Acceleration Trajectory Initial
	for(size_t i = 0 ; i < vec_time_init_uav.size()-2 ; i++){
		init_traj_acc_ = ( ((vec_dist_init_uav[i] / vec_time_init_uav[i+1] ) - (vec_dist_init_uav[i+1]/vec_time_init_uav[i+2])) / (vec_time_init_uav[i+1] +vec_time_init_uav[i+2]) ) + init_traj_acc_; ;
		if (init_traj_acc_max_ < ( ((vec_dist_init_uav[i] / vec_time_init_uav[i+1] )- (vec_dist_init_uav[i+1]/vec_time_init_uav[i+2])) / (vec_time_init_uav[i+1] +vec_time_init_uav[i+2]) ) )
			init_traj_acc_max_ = ( ((vec_dist_init_uav[i] / vec_time_init_uav[i+1] )- (vec_dist_init_uav[i+1]/vec_time_init_uav[i+2])) / (vec_time_init_uav[i+1] +vec_time_init_uav[i+2]) );
	}
	init_traj_acc_mean_ = init_traj_acc_ / (double)(vec_time_init_uav.size()-2.0);

	//Distance Obstacles Initial
	double distance_obs_init_ , distance_obs_init_min_, distance_obs_init_mean_;
	distance_obs_init_ = distance_obs_init_mean_ = 0.0;
	distance_obs_init_min_ = 1000.0;
	for(size_t i = 0 ; i < vec_pose_init_uav.size(); i ++){
		Eigen::Vector3d p_init_ = vec_pose_init_uav[i];
		Eigen::Vector3d nearest_obs_p_ = nn_uav.nearestObstacleVertex(kdt_uav, p_init_, obstacles_points_uav);
		distance_obs_init_ = (p_init_- nearest_obs_p_).norm() + distance_obs_init_;
		if(distance_obs_init_min_ > (p_init_- nearest_obs_p_).norm() )
			distance_obs_init_min_ = (p_init_- nearest_obs_p_).norm();
	}
	distance_obs_init_mean_ = distance_obs_init_ / (double)vec_pose_init_uav.size();

	//Distance Catenary Obstacles Initial
	double distance_obs_cat_init_ , distance_obs_cat_init_min_, distance_obs_cat_init_mean_;
	distance_obs_cat_init_ = distance_obs_cat_init_mean_ = 0.0;
	distance_obs_cat_init_min_ = 1000.0;
	int count_cat_p_init_ = 0;
	int count_coll_between_init_ = 0;
	std::string pos_coll_between_init_;

	for(size_t i = 0 ; i < vec_pose_init_uav.size(); i ++){
		CatenarySolver cS_;
		v_points_catenary_init_.clear();
		geometry_msgs::Point p_reel_ugv;
		p_reel_ugv=getReelPos(vec_pose_init_ugv[i].x(),vec_pose_init_ugv[i].y(),vec_pose_init_ugv[i].z(),vec_rot_ugv[i].x,vec_rot_ugv[i].y,vec_rot_ugv[i].z,vec_rot_ugv[i].w, pos_reel_ugv);
		cS_.setMaxNumIterations(100);
		cS_.solve(p_reel_ugv.x, p_reel_ugv.y, p_reel_ugv.z, vec_pose_init_uav[i].x(), vec_pose_init_uav[i].y(), vec_pose_init_uav[i].z(), vec_len_cat_init[i], v_points_catenary_init_);
		int n_p_cat_dis_ = ceil(1.5*ceil(vec_len_cat_init[i])); // parameter to ignore collsion points in the begining and in the end of catenary
		if (n_p_cat_dis_ < 5)
			n_p_cat_dis_ = 5;
		for (size_t j= 0 ; j < v_points_catenary_init_.size() ; j++){
			if(j > n_p_cat_dis_ && j < v_points_catenary_init_.size()-floor(n_p_cat_dis_/2.0)){
				count_cat_p_init_++;
				Eigen::Vector3d p_cat_init_; 
				p_cat_init_.x()= v_points_catenary_init_[j].x;
				p_cat_init_.y()= v_points_catenary_init_[j].y;
				p_cat_init_.z()= v_points_catenary_init_[j].z;
				Eigen::Vector3d nearest_obs_p = nn_uav.nearestObstacleVertex(kdt_uav, p_cat_init_, obstacles_points_uav);
				distance_obs_cat_init_ = (p_cat_init_- nearest_obs_p).norm() + distance_obs_cat_init_;
				if(distance_obs_cat_init_min_ > (p_cat_init_- nearest_obs_p).norm() )
					distance_obs_cat_init_min_ = (p_cat_init_- nearest_obs_p).norm();
			}
		}
		if ( (i < vec_pose_init_uav.size()-1) && (isObstacleBetweenTwoPoints(vec_pose_init_uav[i], vec_pose_init_uav[i+1])) ){
			count_coll_between_init_++;
			pos_coll_between_init_ = pos_coll_between_init_+"["+std::to_string(i)+"-"+std::to_string(i+1)+"]";
		}
	}
	distance_obs_cat_init_mean_ = distance_obs_cat_init_/ (double)count_cat_p_init_;

	
	// Writing general data for optimized analysis
	double opt_traj_distance_, opt_traj_time_, opt_traj_vel_, opt_traj_vel_max_, opt_traj_vel_mean_, opt_traj_acc_, opt_traj_acc_max_, opt_traj_acc_mean_;
	opt_traj_distance_ = opt_traj_time_ = opt_traj_vel_ = opt_traj_vel_max_ = opt_traj_vel_mean_ = opt_traj_acc_ = opt_traj_acc_max_ = opt_traj_acc_mean_ = 0.0;

	//Length, Time, Velocity Trajectory Optimized
	for (size_t i = 0; i < vec_time_uav_opt.size() -1 ; i++){
		opt_traj_time_ = vec_time_uav_opt[i+1] + opt_traj_time_;
		opt_traj_distance_ = vec_dist_uav_opt[i] + opt_traj_distance_;
		opt_traj_vel_ = vec_vel_uav_opt[i] + opt_traj_vel_;
		if (opt_traj_vel_max_ < vec_vel_uav_opt[i])
			opt_traj_vel_max_ = vec_vel_uav_opt[i];
	}
	opt_traj_vel_mean_ = opt_traj_vel_ / ((double)vec_time_uav_opt.size()-1.0);

	//Acceleration Trajectory Optimized
	for (size_t i = 0; i < vec_acc_uav_opt.size() ; i++){
		opt_traj_acc_ = vec_acc_uav_opt[i] + opt_traj_acc_;
		if (fabs(opt_traj_acc_max_) < fabs(vec_acc_uav_opt[i]))
			opt_traj_acc_max_ = vec_acc_uav_opt[i];
	}
	opt_traj_acc_mean_ = opt_traj_acc_ / (double)vec_acc_uav_opt.size();

	//Distance Point Obstacles
	double distance_obs_opt_ , distance_obs_opt_min_, distance_obs_opt_mean_;
	distance_obs_opt_ = distance_obs_opt_mean_ = 0.0;
	distance_obs_opt_min_ = 1000.0;
	for(size_t i = 0 ; i < vec_pose_uav_opt.size(); i ++){
		Eigen::Vector3d p_opt_ = vec_pose_uav_opt[i];
		Eigen::Vector3d nearest_obs_p_ = nn_uav.nearestObstacleVertex(kdt_uav, p_opt_ , obstacles_points_uav);
		distance_obs_opt_ = (p_opt_- nearest_obs_p_).norm() + distance_obs_opt_;
		if(distance_obs_opt_min_ > (p_opt_- nearest_obs_p_).norm() )
			distance_obs_opt_min_ = (p_opt_- nearest_obs_p_).norm();
	}
	distance_obs_opt_mean_ = distance_obs_opt_ / (double)vec_pose_uav_opt.size();

	//Distance Catenary Obstacles Optimized
	double distance_obs_cat_opt_ , distance_obs_cat_opt_min_, distance_obs_cat_opt_mean_;
	distance_obs_cat_opt_ = distance_obs_cat_opt_mean_ = 0.0;
	distance_obs_cat_opt_min_ = 1000.0;
	int count_cat_p_opt_ = 0;
	int count_coll_between_opt_ = 0;
	std::string pos_coll_between_opt_;

	for(size_t i = 0 ; i < vec_pose_uav_opt.size(); i ++){
		CatenarySolver cS_;
		v_points_catenary_opt_.clear();
		geometry_msgs::Point p_reel_ugv;
		p_reel_ugv=getReelPos(vec_pose_init_ugv[i].x(),vec_pose_init_ugv[i].y(),vec_pose_init_ugv[i].z(),vec_rot_ugv[i].x,vec_rot_ugv[i].y,vec_rot_ugv[i].z,vec_rot_ugv[i].w, pos_reel_ugv);
		cS_.setMaxNumIterations(100);
		cS_.solve(p_reel_ugv.x, p_reel_ugv.y, p_reel_ugv.z, vec_pose_uav_opt[i].x(), vec_pose_uav_opt[i].y(), vec_pose_uav_opt[i].z(), vec_len_cat_opt[i], v_points_catenary_opt_);
		int n_p_cat_dis_ = ceil(1.5*ceil(vec_len_cat_opt[i])); // parameter to ignore collsion points in the begining and in the end of catenary
		if (n_p_cat_dis_ < 5)
			n_p_cat_dis_ = 5;
		for (size_t j= 0 ; j < v_points_catenary_opt_.size() ; j++){
			if(j > n_p_cat_dis_ && j < v_points_catenary_opt_.size()-floor(n_p_cat_dis_/2.0)){
				count_cat_p_opt_++;
				Eigen::Vector3d p_cat_opt_; 
				p_cat_opt_.x()= v_points_catenary_opt_[j].x;
				p_cat_opt_.y()= v_points_catenary_opt_[j].y;
				p_cat_opt_.z()= v_points_catenary_opt_[j].z;
				Eigen::Vector3d nearest_obs_p = nn_uav.nearestObstacleVertex(kdt_uav, p_cat_opt_ , obstacles_points_uav);
				distance_obs_cat_opt_ = (p_cat_opt_- nearest_obs_p).norm() + distance_obs_cat_opt_;
				// if ((p_cat_opt_- nearest_obs_p).norm() > 100.0)
				// 	printf ( "state[%lu]=[%f %f %f] point_cat[%lu]=[%f %f %f]\n",i,vec_pose_uav_opt[i].x(), vec_pose_uav_opt[i].y(), vec_pose_uav_opt[i].z(),j,p_cat_opt_.x(),p_cat_opt_.y(),p_cat_opt_.z());
				if(distance_obs_cat_opt_min_ > (p_cat_opt_- nearest_obs_p).norm() )
					distance_obs_cat_opt_min_ = (p_cat_opt_- nearest_obs_p).norm();
			}
		// printf("Obstacles Optimized: state=[%lu/%lu] vec_pose_uav_opt.size=[%lu] v_points_catenary_opt_.size=[%lu] count_cat_p_opt = [%i]\n",i, vec_pose_uav_opt.size(),vec_pose_uav_opt.size(),v_points_catenary_opt_.size(),count_cat_p_opt_);
		}
	
		if ( (i < vec_pose_uav_opt.size()-1) && (isObstacleBetweenTwoPoints(vec_pose_uav_opt[i], vec_pose_uav_opt[i+1])) ){
			count_coll_between_opt_++;
			pos_coll_between_opt_ = pos_coll_between_opt_+"["+std::to_string(i)+"-"+std::to_string(i+1)+"]";
		}
	}
	distance_obs_cat_opt_mean_ = distance_obs_cat_opt_/ (double)count_cat_p_opt_;


	if (ofs.is_open()) {
		std::cout << "Saving data in output file: " << output_file << std::endl;
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
		std::cout << "Couldn't be open the output data file" << std::endl;
	}
	ofs.close();
}

inline geometry_msgs::Point DataManagement::getReelPos(const float px_, const float py_, const float pz_,const float qx_, const float qy_, const float qz_, const float qw_, geometry_msgs::Point p_reel_)
{
	geometry_msgs::Point ret;

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

inline bool DataManagement::isObstacleBetweenTwoPoints(Eigen::Vector3d pose_uav_opt_1, Eigen::Vector3d pose_uav_opt_2)
{
	octomap::point3d r_;
	octomap::point3d s_(pose_uav_opt_1.x() , pose_uav_opt_1.y() , pose_uav_opt_1.z() ); //start for rayCast
	octomap::point3d d_(pose_uav_opt_2.x()-pose_uav_opt_1.x() , pose_uav_opt_2.y()-pose_uav_opt_1.y() , pose_uav_opt_2.z()-pose_uav_opt_1.z() ); //direction for rayCast
    bool r_cast_coll = octotree->castRay(s_, d_, r_);

	double dist12_ = sqrt ( (pose_uav_opt_2.x()-pose_uav_opt_1.x())*(pose_uav_opt_2.x()-pose_uav_opt_1.x())+
					   		(pose_uav_opt_2.y()-pose_uav_opt_1.y())*(pose_uav_opt_2.y()-pose_uav_opt_1.y())+ 
					   		(pose_uav_opt_2.z()-pose_uav_opt_1.z())*(pose_uav_opt_2.z()-pose_uav_opt_1.z()) );
	
	double distObs_ = sqrt ((pose_uav_opt_1.x()-r_.x())*(pose_uav_opt_1.x()-r_.x())+
					   		(pose_uav_opt_1.y()-r_.y())*(pose_uav_opt_1.y()-r_.y())+ 
					   		(pose_uav_opt_1.z()-r_.z())*(pose_uav_opt_1.z()-r_.z()) );
	
	if (r_cast_coll && distObs_ <= dist12_)
		return true;
	else
		return false;
}


#endif