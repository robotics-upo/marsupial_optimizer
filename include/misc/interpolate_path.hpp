#ifndef INTERPOLATE_PATH_HPP
#define INTERPOLATE_PATH_HPP

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

#include "Eigen/Core"
#include <Eigen/StdVector>

#include "catenary_checker/bisection_catenary_3D.h"
#include "catenary_checker/near_neighbor.hpp"
#include "catenary_checker/grid3d.hpp"


using namespace std;

class InterpolatePath
{
	public:
		InterpolatePath();
		// ~InterpolatePath(){};
		virtual void initInterpolatePath(int c_f_i_ugv_, int c_f_f_ugv_, int c_f_uav_, bool f_l_p_ugv_, double d_c_o_, 
										 bool u_d_f_, int s_, double l_max_, double ws_z_min_, double map_r_, 
										 geometry_msgs::TransformStamped p_r_l_, Grid3d* grid_3D_);
		
		/*
		Allow to identify the states that are located in the same position. Return a path with new states computed in interpolateFixedPointsPath method
		*/
		virtual void getInitialGlobalPath(trajectory_msgs::MultiDOFJointTrajectory _path, vector<float> &vl_init_,
										  vector<geometry_msgs::Vector3> &v_ugv_, vector<geometry_msgs::Vector3> &v_uav_,
										  vector<geometry_msgs::Quaternion> &v_q_ugv_, vector<geometry_msgs::Quaternion> &v_q_uav_);
		/*
		It computes new positions for states in the same location. Allow a relocation of the states
		*/
		virtual void interpolateFixedPointsPath(vector<geometry_msgs::Vector3> &v_inter_ , int mode_);
		virtual bool computeInitialCatenary(int p_, double &l_cat_);
		virtual double getPointDistanceFullMap(bool use_dist_func_, geometry_msgs::Vector3 p_);
		virtual geometry_msgs::Vector3 getReelPoint(const float px_, const float py_, const float pz_,const float qx_, const float qy_, const float qz_, const float qw_);
	
		Grid3d* grid_3D;

		int count_fix_points_initial_ugv, count_fix_points_final_ugv, count_fix_points_uav;
		bool fix_last_position_ugv, use_distance_function;
		double distance_catenary_obstacle,  length_tether_max, ws_z_min, map_resolution;

		vector<geometry_msgs::Vector3> vec_pose_init_ugv, vec_pose_init_uav;
		vector<geometry_msgs::Quaternion> vec_init_rot_ugv;
		vector<float> vec_len_cat_init;
		vector<int> vec_fix_status_ugv_prepross, v_interpolated_positions;
		geometry_msgs::TransformStamped pose_reel_local;

	protected:

	private:

};

inline InterpolatePath::InterpolatePath()
{
}

inline void InterpolatePath::initInterpolatePath(int c_f_i_ugv_, int c_f_f_ugv_, int c_f_uav_, bool f_l_p_ugv_, double d_c_o_, 
												 bool u_d_f_, int s_, double l_max_, double ws_z_min_, double map_r_, 
												 geometry_msgs::TransformStamped p_r_l_, Grid3d* grid_3D_)
{
	count_fix_points_initial_ugv = c_f_i_ugv_;
	count_fix_points_final_ugv = c_f_f_ugv_;
	count_fix_points_uav = c_f_uav_;

	fix_last_position_ugv = f_l_p_ugv_;
	distance_catenary_obstacle = d_c_o_;
	use_distance_function = u_d_f_;
	length_tether_max = l_max_;
	ws_z_min = ws_z_min_;
	map_resolution = map_r_;
	pose_reel_local = p_r_l_;
	grid_3D = grid_3D_;

	vec_fix_status_ugv_prepross.clear();
	vec_fix_status_ugv_prepross.assign(s_,0);
}

inline void InterpolatePath::getInitialGlobalPath(trajectory_msgs::MultiDOFJointTrajectory _path, vector<float> &vl_init_,
										   vector<geometry_msgs::Vector3> &v_ugv_, vector<geometry_msgs::Vector3> &v_uav_,
										   vector<geometry_msgs::Quaternion> &v_q_ugv_, vector<geometry_msgs::Quaternion> &v_q_uav_)
{
	float x_ugv_, y_ugv_, z_ugv_, x_uav_, y_uav_, z_uav_;
	int aux_cont_ugv_, aux_cont_uav_;
	double D_ugv_, D_uav_;
	geometry_msgs::Vector3 p_uav_ , p_ugv_;
	geometry_msgs::Quaternion qt_;

	aux_cont_ugv_ = aux_cont_uav_ = 0;
    D_ugv_ = D_uav_ = 0.0;
	vec_len_cat_init.clear();
	v_ugv_.clear();	v_uav_.clear(); 
	v_q_ugv_.clear(); v_q_uav_.clear();
	vec_pose_init_ugv.clear(); vec_pose_init_uav.clear();

	vec_len_cat_init = vl_init_;
	v_interpolated_positions.clear();
	v_interpolated_positions.push_back(0); //First position always is fixed
    for (size_t i = 0; i < _path.points.size()-1; i++)
    {
		// Get position and rotation vector for UGV
		x_ugv_ = _path.points.at(i+1).transforms[0].translation.x - _path.points.at(i).transforms[0].translation.x;
		y_ugv_ = _path.points.at(i+1).transforms[0].translation.y - _path.points.at(i).transforms[0].translation.y;
		z_ugv_ = _path.points.at(i+1).transforms[0].translation.z - _path.points.at(i).transforms[0].translation.z;
		
		// Fallowing lines to count the number of UAV initial nodes in the same position 
        D_ugv_ = sqrt(x_ugv_ * x_ugv_ + y_ugv_ * y_ugv_ + z_ugv_ * z_ugv_);
		if (D_ugv_ < 0.001){
			if(aux_cont_ugv_ == i){  // To count the consecutive points that keep stable in the start point
				count_fix_points_initial_ugv = count_fix_points_initial_ugv + 1;
				aux_cont_ugv_++;
			}
			count_fix_points_final_ugv = count_fix_points_final_ugv + 1;
			v_interpolated_positions.push_back(1);
		}else{ // To make count = 1 in case is not fix point
			count_fix_points_final_ugv = 1;
			v_interpolated_positions.push_back(0);
		}
		// Get position and rotation vector for UAV
		x_uav_ = _path.points.at(i+1).transforms[1].translation.x - _path.points.at(i).transforms[1].translation.x;
		y_uav_ = _path.points.at(i+1).transforms[1].translation.y - _path.points.at(i).transforms[1].translation.y;
		z_uav_ = _path.points.at(i+1).transforms[1].translation.z - _path.points.at(i).transforms[1].translation.z;
		
		// Fallowing lines to count the number of UAV initial nodes in the same position 
        D_uav_ = sqrt(x_uav_ * x_uav_ + y_uav_ * y_uav_ + z_uav_ * z_uav_);
		if (D_uav_ < 0.001){
			if(aux_cont_uav_ == i){  // To count the consecutive points that keep stable in the start point
				count_fix_points_uav = count_fix_points_uav + 1;
				aux_cont_uav_++;
			}
			v_interpolated_positions[i] = 1;
		}
		// Get position and rotation vector for UGV
		p_ugv_.x = _path.points.at(i).transforms[0].translation.x;
		p_ugv_.y = _path.points.at(i).transforms[0].translation.y;
		p_ugv_.z = _path.points.at(i).transforms[0].translation.z;
		v_ugv_.push_back(p_ugv_);
		qt_.x = _path.points.at(i).transforms[0].rotation.x;
		qt_.y = _path.points.at(i).transforms[0].rotation.y;
		qt_.z = _path.points.at(i).transforms[0].rotation.z;
		qt_.w = _path.points.at(i).transforms[0].rotation.w;
		v_q_ugv_.push_back(qt_);
		// Get position and rotation vector for UAV
		p_uav_.x = _path.points.at(i).transforms[1].translation.x;
		p_uav_.y = _path.points.at(i).transforms[1].translation.y;
		p_uav_.z = _path.points.at(i).transforms[1].translation.z;
		v_uav_.push_back(p_uav_);
		qt_.x = _path.points.at(i).transforms[1].rotation.x;
		qt_.y = _path.points.at(i).transforms[1].rotation.y;
		qt_.z = _path.points.at(i).transforms[1].rotation.z;
		qt_.w = _path.points.at(i).transforms[1].rotation.w;
		v_q_uav_.push_back(qt_);
    }
	v_interpolated_positions[_path.points.size()-1] = 0; //Last position always is fixed


	// Get position and rotation vector for UGV
	p_ugv_.x = _path.points.at(_path.points.size()-1).transforms[0].translation.x;
	p_ugv_.y = _path.points.at(_path.points.size()-1).transforms[0].translation.y;
	p_ugv_.z = _path.points.at(_path.points.size()-1).transforms[0].translation.z;
	v_ugv_.push_back(p_ugv_);
	qt_.x = _path.points.at(_path.points.size()-1).transforms[0].rotation.x;
	qt_.y = _path.points.at(_path.points.size()-1).transforms[0].rotation.y;
	qt_.z = _path.points.at(_path.points.size()-1).transforms[0].rotation.z;
	qt_.w = _path.points.at(_path.points.size()-1).transforms[0].rotation.w;
	v_q_ugv_.push_back(qt_);
	// Get position and rotation vector for UAV
	p_uav_.x = _path.points.at(_path.points.size()-1).transforms[1].translation.x;
	p_uav_.y = _path.points.at(_path.points.size()-1).transforms[1].translation.y;
	p_uav_.z = _path.points.at(_path.points.size()-1).transforms[1].translation.z;
	v_uav_.push_back(p_uav_);
	qt_.x = _path.points.at(_path.points.size()-1).transforms[1].rotation.x;
	qt_.y = _path.points.at(_path.points.size()-1).transforms[1].rotation.y;
	qt_.z = _path.points.at(_path.points.size()-1).transforms[1].rotation.z;
	qt_.w = _path.points.at(_path.points.size()-1).transforms[1].rotation.w;
	v_q_uav_.push_back(qt_);

	if(!fix_last_position_ugv)
		count_fix_points_final_ugv = 0;

	for (size_t i = 0 ; i <  v_ugv_.size() ; i++){
		vec_pose_init_ugv.push_back(v_ugv_[i]);
	}
	for (size_t i = 0 ; i <  v_uav_.size() ; i++){
		vec_pose_init_uav.push_back(v_uav_[i]);
	}

	vec_init_rot_ugv = v_q_ugv_;

	interpolateFixedPointsPath(v_ugv_,0);
	interpolateFixedPointsPath(v_uav_,1);
	vl_init_.clear();
	vl_init_ = vec_len_cat_init;
}

inline void InterpolatePath::interpolateFixedPointsPath(vector<geometry_msgs::Vector3> &v_inter_ , int mode_)
{
	// mode_==0 for UGV , mode_==1 for UAV
	vector<geometry_msgs::Vector3> vec_pose_init_aux_;
	vector<geometry_msgs::Vector3> vec_pose_init_;
	vector<bool> is_pos_interpolated_;
	vector<float> vec_length_aux_;
	vec_length_aux_.clear();
	vec_pose_init_aux_.clear();
	v_inter_.clear(); is_pos_interpolated_.clear();
	int count_fix_points_;

	if (mode_==0){
		vec_pose_init_ = vec_pose_init_ugv;
		count_fix_points_ = count_fix_points_initial_ugv;
	}else if(mode_==1){
		vec_pose_init_ = vec_pose_init_uav;
		count_fix_points_ = count_fix_points_uav;
	}else
		ROS_ERROR("WARNNING : In method interpolateFixedPointsPath from optimizer was not set the MODE, this is going to run into trouble");

	geometry_msgs::Vector3 pos_ugv_;
	int count_ = 1;
	int init_pos_ = 0;
	int final_pos_ = 0;
	int final_pos_aux = -1;
	int group_ = 1;
	for (size_t i = 0; i < vec_pose_init_.size()-1; i++)
    {
		// "if" make two task: First, in case there are some points in the same first position, they are keeped fix. Second, for points after first 
		// position fixed, they are interpolated
		if(i>=count_fix_points_ && i < (vec_pose_init_.size()-count_fix_points_final_ugv) ){
			double x_ugv_ = vec_pose_init_[i+1].x - vec_pose_init_[i].x;
			double y_ugv_ = vec_pose_init_[i+1].y - vec_pose_init_[i].y;
			double z_ugv_ = vec_pose_init_[i+1].z - vec_pose_init_[i].z;
			double D_ugv_ = sqrt(x_ugv_ * x_ugv_ + y_ugv_ * y_ugv_ + z_ugv_ * z_ugv_);
			group_ = 1;

			if (D_ugv_ < 0.001){ // Counts points that are in the same position
				if (count_ == 1)	// Check if first position has more than one point
					init_pos_ = i ;
				count_++;
			}
			else if (D_ugv_ > 0.001 && count_ > 1){ // Points, that are not in the first position but they share position, are interpolated  	
				final_pos_ = i +1 ;
				double x_ = vec_pose_init_[final_pos_].x - vec_pose_init_[init_pos_].x;
				double y_ = vec_pose_init_[final_pos_].y - vec_pose_init_[init_pos_].y;
				double z_ = vec_pose_init_[final_pos_].z - vec_pose_init_[init_pos_].z;
				for(int j = 0; j < count_; j++){
					pos_ugv_.x = vec_pose_init_[init_pos_].x + (x_/count_)*j;
					pos_ugv_.y = vec_pose_init_[init_pos_].y + (y_/count_)*j;
					pos_ugv_.z = vec_pose_init_[init_pos_].z + (z_/count_)*j;
					vec_pose_init_aux_.push_back(pos_ugv_);
					is_pos_interpolated_.push_back(true);
					if (mode_==0){
						if (final_pos_aux != final_pos_){
							final_pos_aux = final_pos_;
							group_++;
						}
						vec_fix_status_ugv_prepross[init_pos_+j]=group_;
					}
				}
				count_ = 1;
			}
			else{
				vec_pose_init_aux_.push_back(vec_pose_init_[i]);
				is_pos_interpolated_.push_back(false);
			}
		}
		else{
			vec_pose_init_aux_.push_back(vec_pose_init_[i]);
			is_pos_interpolated_.push_back(false);
			if(i < count_fix_points_){
				vec_fix_status_ugv_prepross[i]=group_;
				group_++;
			}
		}

		if ((i== vec_pose_init_.size()-2 )&& (count_ > 1)){ // Interpolates the last position 
			double x_ugv_ = vec_pose_init_[vec_pose_init_.size()-1].x - vec_pose_init_[vec_pose_init_.size()-(1+count_)].x;
			double y_ugv_ = vec_pose_init_[vec_pose_init_.size()-1].y - vec_pose_init_[vec_pose_init_.size()-(1+count_)].y;
			double z_ugv_ = vec_pose_init_[vec_pose_init_.size()-1].z - vec_pose_init_[vec_pose_init_.size()-(1+count_)].z;
			group_ = 0;
			for(int j = 1; j < count_; j++){
				pos_ugv_.x = vec_pose_init_[vec_pose_init_.size()-(1+count_)].x + (x_ugv_/count_)*j;
				pos_ugv_.y = vec_pose_init_[vec_pose_init_.size()-(1+count_)].y + (y_ugv_/count_)*j;
				pos_ugv_.z = vec_pose_init_[vec_pose_init_.size()-(1+count_)].z + (z_ugv_/count_)*j;
				vec_pose_init_aux_.push_back(pos_ugv_);
				is_pos_interpolated_.push_back(true);
				vec_fix_status_ugv_prepross[vec_pose_init_.size()-(j)]= count_- group_;
				group_++;
			}
				vec_fix_status_ugv_prepross[vec_pose_init_.size()-count_]= count_- group_;
		}
    }
	
	int num_pos_ = vec_pose_init_.size()-1;
	vec_pose_init_aux_.push_back(vec_pose_init_[num_pos_]);
	is_pos_interpolated_.push_back(false); // always false because is the last point

	//Set new values in vec_pos_init_ugv or vec_pos_init_ugv
	if (mode_==0){
		vec_pose_init_ugv.clear();
		vec_pose_init_ugv = vec_pose_init_aux_;
		v_inter_ = vec_pose_init_aux_;
	}
	else if(mode_==1){
		vec_pose_init_uav.clear();
		vec_pose_init_uav = vec_pose_init_aux_;
		v_inter_ = vec_pose_init_aux_;
	}
	else
		ROS_ERROR("WARNNING : In method interpolateFixedPointsPath from optimizer was not set the MODE, this is going to run into trouble");

	if (mode_ == 1){
		for (int m=0 ; m < is_pos_interpolated_.size(); m++){
			if(is_pos_interpolated_[m]){
				double length_;
				if ( computeInitialCatenary(m, length_)){
					vec_length_aux_.push_back(length_);
					}
				else{
					vec_length_aux_.push_back(vec_len_cat_init[m]);
					ROS_ERROR("not catenary interpolated : length[%i]=%f",m,length_);
				}
			}
			else
				vec_length_aux_.push_back(vec_len_cat_init[m]);
		}
		vec_len_cat_init.clear();
		vec_len_cat_init = vec_length_aux_;
	}
}

inline bool InterpolatePath::computeInitialCatenary(int p_, double &l_cat_)
{
	geometry_msgs::Vector3 p_reel_;
    std::vector<geometry_msgs::Vector3> p_catenary_;

	bisectionCatenary bc;

    p_reel_ = getReelPoint(vec_pose_init_ugv[p_].x , vec_pose_init_ugv[p_].y , vec_pose_init_ugv[p_].z,
                           vec_init_rot_ugv[p_].x, vec_init_rot_ugv[p_].y, vec_init_rot_ugv[p_].z, vec_init_rot_ugv[p_].w);
                    
    double dist_ = sqrt(pow(p_reel_.x - vec_pose_init_uav[p_].x,2) + 
                        pow(p_reel_.y - vec_pose_init_uav[p_].y,2) + 
                        pow(p_reel_.z - vec_pose_init_uav[p_].z,2));
	double dist_init_final_;
	
	if (dist_ < 4.0)
		dist_init_final_ = 1.010 * dist_;
	else
		dist_init_final_ = 1.005 * dist_;
    double delta_ = 0.0;	//Initial Value
    bool check_catenary = true;
    bool hard_collision_ = false;
	bool increase_catenary;
    double security_dis_ca_ = distance_catenary_obstacle;
                    
	do{
		increase_catenary = false;
	    p_catenary_.clear();
		
	    l_cat_ = dist_init_final_* (1.0 + delta_);
	    if (l_cat_ > length_tether_max){
	        check_catenary = false;
			printf("interpolated_cat bigger than maximum length\n");
			hard_collision_ = true;
	        break;
	    }

	    bool just_one_axe = bc.configBisection(l_cat_, p_reel_.x, p_reel_.y, p_reel_.z, vec_pose_init_uav[p_].x, vec_pose_init_uav[p_].y, vec_pose_init_uav[p_].z);
		bc.getPointCatenary3D(p_catenary_, false);
		double d_min_point_cat = 100000;
		if (p_catenary_.size() > 2){
		    for (size_t i = 0 ; i < p_catenary_.size() ; i++){
		        geometry_msgs::Vector3 point_cat, p_in_cat_;
		        if (p_catenary_[i].z < ws_z_min*map_resolution + ((1*map_resolution)+security_dis_ca_)){
		            check_catenary = false;
					printf("interpolated_cat_collison in the floor\n");
					hard_collision_ = true;
		            break;
		        }
		        p_in_cat_.x = p_catenary_[i].x;
		        p_in_cat_.y = p_catenary_[i].y;
		        p_in_cat_.z = p_catenary_[i].z;
				double dist_cat_obs = getPointDistanceFullMap(use_distance_function, p_in_cat_);
		        // double dist_cat_obs;
		        // bool is_into_ = grid_3D->isIntoMap(p_in_cat_.x,p_in_cat_.y,p_in_cat_.z);
		        // if(is_into_)
		        //     dist_cat_obs =  grid_3D->getPointDist((double)p_in_cat_.x,(double)p_in_cat_.y,(double)p_in_cat_.z) ;
	            // else
	            //     dist_cat_obs = -1.0;

	            if (d_min_point_cat > dist_cat_obs){
	                d_min_point_cat = dist_cat_obs;
	            }
	            if (dist_cat_obs < security_dis_ca_){
	                delta_ = delta_ + 0.001;
	                increase_catenary = true;
	                break;
	            }
	            point_cat.x = p_catenary_[i].x;
	            point_cat.y = p_catenary_[i].y;
	            point_cat.z = p_catenary_[i].z;
	        }
	        if (check_catenary && !increase_catenary){
	            check_catenary = false;
	        }
	    }
	    else{
	        check_catenary = false;
	    }
	}while (check_catenary);

	if (!check_catenary && !hard_collision_)
		return true;
	else
		return false;
}

inline double InterpolatePath::getPointDistanceFullMap(bool use_dist_func_, geometry_msgs::Vector3 p_)
{
	double dist;

	if(use_dist_func_){
		bool is_into_ = grid_3D->isIntoMap(p_.x,p_.y,p_.z);
		if(is_into_)
			dist =  grid_3D->getPointDist((double)p_.x,(double)p_.y,(double)p_.z) ;
		else
			dist = -1.0;
	}
	// else{
	// 	Eigen::Vector3d obs_, pos_;
	// 	pos_.x() = p_.x;
	// 	pos_.y() = p_.y; 
	// 	pos_.z() = p_.z; 
	// 	obs_ = nn_uav.nearestObstacleMarsupial(nn_uav.kdtree, pos_, nn_uav.obs_points);
	// 	dist = sqrt(pow(obs_.x()-pos_.x(),2) + pow(obs_.y()-pos_.y(),2) + pow(obs_.z()-pos_.z(),2));
	// }
	
	return dist;
}

inline geometry_msgs::Vector3 InterpolatePath::getReelPoint(const float px_, const float py_, const float pz_,const float qx_, const float qy_, const float qz_, const float qw_)
{
	geometry_msgs::Vector3 ret;

	double roll_, pitch_, yaw_;
	tf::Quaternion q_(qx_,qy_,qz_,qw_);
	tf::Matrix3x3 M_(q_);	
	M_.getRPY(roll_, pitch_, yaw_);

	double lengt_vec =  sqrt(pose_reel_local.transform.translation.x*pose_reel_local.transform.translation.x + pose_reel_local.transform.translation.y*pose_reel_local.transform.translation.y);
	ret.x = px_ + lengt_vec *cos(yaw_); 
	ret.y = py_ + lengt_vec *sin(yaw_);
	ret.z = pz_ + pose_reel_local.transform.translation.z ;

	return ret;
}

#endif