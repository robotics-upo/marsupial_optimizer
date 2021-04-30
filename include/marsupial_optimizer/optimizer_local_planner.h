/*
Optimizer Local Planner based in g2o for Marsupial Robotics Configuration 
Simon Martinez Rozas, 2020 
Service Robotics Lab, University Pablo de Olavide , Seville, Spain 
 */

#ifndef _OPTIMIZER_LOCAL_PLANNER_CERES_H_
#define _OPTIMIZER_LOCAL_PLANNER_CERES_H_

#include <Eigen/StdVector>
#include <random>
#include <stdint.h>

#include <iostream>
#include <vector>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

#include "marsupial_optimizer/ceres_contrain_equidistance_ugv.hpp"
#include "marsupial_optimizer/ceres_contrain_equidistance_uav.hpp"
#include "marsupial_optimizer/ceres_contrain_obstacles_ugv.hpp"
#include "marsupial_optimizer/ceres_contrain_obstacles_uav.hpp"
#include "marsupial_optimizer/ceres_contrain_obstacles_through_uav.hpp"
#include "marsupial_optimizer/ceres_contrain_traversability_ugv.hpp"
#include "marsupial_optimizer/ceres_contrain_kinematics_ugv.hpp"
#include "marsupial_optimizer/ceres_contrain_rotation_ugv.hpp"
#include "marsupial_optimizer/ceres_contrain_kinematics_uav.hpp"
#include "marsupial_optimizer/ceres_contrain_time.hpp"
#include "marsupial_optimizer/ceres_contrain_velocity.hpp"
#include "marsupial_optimizer/ceres_contrain_acceleration.hpp"
#include "marsupial_optimizer/ceres_contrain_catenary.hpp"
#include "marsupial_optimizer/ceres_contrain_dynamic_catenary.hpp"

#include "misc/catenary_solver_ceres.hpp"
#include "misc/near_neighbor.hpp"
#include "misc/data_management.hpp"
// #include "misc/bisection_catenary_3D.h"
// #include "misc/marker_publisher.hpp"

//ROS
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <octomap_msgs/Octomap.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <upo_actions/ExecutePathAction.h>
#include <upo_actions/NavigateAction.h>
#include <upo_actions/Navigate3DAction.h>
#include "marsupial_optimizer/OptimizationParamsConfig.h"

#define PRINTF_REGULAR "\x1B[0m"
#define PRINTF_RED "\x1B[31m"
#define PRINTF_GREEN "\x1B[32m"
#define PRINTF_YELLOW "\x1B[33m"
#define PRINTF_BLUE "\x1B[34m"
#define PRINTF_MAGENTA "\x1B[35m"
#define PRINTF_CYAN "\x1B[36m"
#define PRINTF_WHITE "\x1B[37m"

using namespace Eigen;
using namespace std;
// using namespace g2o;

using ceres::AutoDiffCostFunction;
using ceres::NumericDiffCostFunction;
using ceres::CENTRAL;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;


struct parameterBlockPos{
	double parameter[4];
};

struct parameterBlockRot{
	double parameter[9];
};

struct parameterBlockTime{
	double parameter[3];
};

struct parameterBlockLength{
	double parameter[2];
};

class OptimizerLocalPlanner
{
    typedef actionlib::SimpleActionServer<upo_actions::ExecutePathAction> ExecutePathServer;
    typedef actionlib::SimpleActionClient<upo_actions::NavigateAction> NavigateClient;
    typedef actionlib::SimpleActionClient<upo_actions::Navigate3DAction> Navigate3DClient;

public:

	// =========== Function declarations =============
	OptimizerLocalPlanner(tf2_ros::Buffer *tfBuffer_);
	// ~OptimizerLocalPlanner();
	void setupOptimizer();
	void initializeSubscribers();
	void initializePublishers();
	void readOctomapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
	void readPointCloudTraversabilityUGVCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
	void readPointCloudObstaclesUGVCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
	void configServices();
	void executeOptimizerPathPreemptCB();
	void executeOptimizerPathGoalCB();
	void dynRecCb(marsupial_optimizer::OptimizationParamsConfig &config, uint32_t level);
	void collisionMapCallBack(const octomap_msgs::OctomapConstPtr &msg);
	void deleteMarkersCallBack(const std_msgs::BoolConstPtr &msg);
	void calculateDistanceVertices(vector<double> &_v_D_ugv,vector<double> &_v_D_uav);
	void getTemporalState(vector<double> &_time_ugv, vector<double> &_time_uav);
	void getPointsFromGlobalPath(trajectory_msgs::MultiDOFJointTrajectory _path, vector<Eigen::Vector3d> &v_ugv_, vector<Eigen::Vector3d> &v_uav_);
	void getDataForOptimizerAnalysis();
	void publishOptimizedTraj(vector<Eigen::Vector3d> pos_ugv_opt_, vector<Eigen::Vector3d> pos_uav_opt_);
	geometry_msgs::Point getReelPoint(const float px_, const float py_, const float pz_,const float qx_, const float qy_, const float qz_, const float qw_, geometry_msgs::Point p_reel_);

	upo_actions::ExecutePathResult action_result;

	// ============= Global Variables ================

	ros::NodeHandlePtr nh;
	tf2_ros::Buffer *tfBuffer;

	Solver::Options options;
  	Solver::Summary summary;
	
	tf::TransformListener listener;

	std::string path, name_output_file;
	int n_iter_opt;	//Iterations Numbers of Optimizer
	double distance_catenary_obstacle,min_distance_add_new_point,dynamic_catenary, initial_distance_states_ugv, initial_distance_states_uav;
	double w_alpha_uav, w_alpha_ugv, w_beta_uav, w_beta_ugv, w_iota_uav, w_theta_ugv, w_gamma_uav, w_gamma_ugv, w_delta, w_epsilon, w_zeta, w_eta, w_lambda;

	NearNeighbor nn_uav; // Kdtree used for Catenary and UAV
	NearNeighbor nn_trav, nn_ugv_obs;
	MarkerPublisher mp_;
	DataManagement dm_;

	//! Manage Data Vertices and Edges
    std::unique_ptr<ExecutePathServer> execute_path_srv_ptr;
    std::unique_ptr<Navigate3DClient> navigation3DClient;
    std::unique_ptr<tf::TransformListener> tf_list;
    std::unique_ptr<tf::TransformListener> tf_list_ptr;
	bool use3d;
    bool mapReceived;
    bool debug;
    bool showConfig;

    trajectory_msgs::MultiDOFJointTrajectory globalTrajectory, localTrajectory;
    ros::Time start_time;

	visualization_msgs::MarkerArray points_uav_marker, lines_uav_marker, points_ugv_marker, lines_ugv_marker;
	typedef visualization_msgs::Marker RVizMarker;

	std::string action_name_;

	ros::Subscriber octomap_ws_sub_,point_cloud_ugv_traversability_sub_, point_cloud_ugv_obstacles_sub_, clean_markers_sub_, local_map_sub;
	ros::Publisher traj_marker_ugv_pub_, traj_marker_uav_pub_, trajPub;

	ros::Publisher catenary_marker_pub_;
	visualization_msgs::MarkerArray catenary_marker;

	geometry_msgs::Point obs_oct;

	bool verbose_optimizer;

	void setStraightTrajectory(double x1, double y1, double z1, double x2, double y2, double z2, int n_v_u_);	//Function to create a straight line from point A to B

	vector<Eigen::Vector3d> new_path_ugv, new_path_uav;
	vector<parameterBlockPos> statesPosUGV;
	vector<parameterBlockPos> statesPosUAV;
	vector<parameterBlockRot> statesRot;
	vector<parameterBlockTime> statesTime;
	vector<parameterBlockLength> statesLength;
	vector<geometry_msgs::Quaternion> vec_rot_ugv, vec_rot_uav;

	vector<double> vec_dist_init_ugv, vec_dist_init_uav;
	vector<double> vec_time_init_ugv, vec_time_init_uav;
	vector<float> vec_len_cat_init, vec_len_cat_opt;
	vector<Eigen::Vector3d> vec_pose_ugv_opt, vec_pose_uav_opt; 
	vector<Eigen::Vector3d> vec_pose_init_ugv, vec_pose_init_uav;
	vector<double> vec_time_ugv_opt, vec_time_uav_opt;
	
    double pos_reel_x, pos_reel_y, pos_reel_z;
	ros::Time start_time_opt, final_time_opt;
	int scenario_number, num_pos_initial, num_goal;
	bool write_data_for_analysis;
	geometry_msgs::Point pos_reel_ugv;
	double length_tether_max;


private:
	void resetFlags();
	void cleanVectors();
	double global_path_length;
	double distance_obstacle_uav,distance_obstacle_ugv, initial_velocity_ugv, initial_velocity_uav, angle_min_traj, initial_acceleration_ugv, initial_acceleration_uav;
	geometry_msgs::Point ugv_pos_catenary;
};

#endif