#ifndef _IMPORT_PATH_HPP_
#define _IMPORT_PATH_HPP_


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


using namespace std;

class ImportPath
{

public:
    ImportPath();
    ImportPath(const std::string &path_and_name_file_, upo_actions::ExecutePathGoal &g_);

    trajectory_msgs::MultiDOFJointTrajectory trajectory;
    std::vector<float> tether_length_vector;

    geometry_msgs::Pose init_uav_pose, init_ugv_pose;

};

#endif
