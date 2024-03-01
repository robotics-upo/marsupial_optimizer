#ifndef _MANAGE_COMPUTED_PATH_HPP_
#define _MANAGE_COMPUTED_PATH_HPP_


#include <ctime>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
// #include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <yaml-cpp/yaml.h>

#include <upo_actions/ExecutePathAction.h>
#include "marsupial_optimizer/marsupial_trajectory_optimized.h"  //create a msg

using namespace std;

class ManagePath
{

public:
    ManagePath();
    ManagePath(const std::string &path_and_name_file_, upo_actions::ExecutePathGoal &g_);
    void exportOptimizedPath(vector<geometry_msgs::Vector3> &v_ugv_, vector<geometry_msgs::Vector3> &v_uav_, 
									 vector<geometry_msgs::Quaternion> &v_r_ugv_, vector<geometry_msgs::Quaternion> &v_r_uav_, vector<float> &v_l_,
									 string path_mission_file_);
    void publishOptimizedTraj(vector<geometry_msgs::Vector3> v_ugv_, vector<geometry_msgs::Vector3> v_uav_, 
							vector<geometry_msgs::Quaternion> v_r_ugv_, vector<geometry_msgs::Quaternion> v_r_uav_, 
							vector<float> v_l_, vector<double> v_t_, marsupial_optimizer::marsupial_trajectory_optimized &msg_);

    trajectory_msgs::MultiDOFJointTrajectory trajectory;
    std::vector<float> tether_length_vector;

    geometry_msgs::Pose init_uav_pose, init_ugv_pose;

};

#endif