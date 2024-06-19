/*
Optimizer Local Planner based in g2o for Marsupial Robotics Configuration 
Simon Martinez Rozas, 2020 
Service Robotics Lab, University Pablo de Olavide , Seville, Spain 
 */

#ifndef _TEST_TETHER_CONSTRAINT_CERES_H_
#define _TEST_TETHER_CONSTRAINT_CERES_H_

#include <Eigen/StdVector>
#include <random>
#include <stdint.h>

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <yaml-cpp/yaml.h>
#include <ctime>

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

#include "marsupial_optimizer/ceres_constraint_tether_obstacles.hpp"
#include "marsupial_optimizer/ceres_constraint_tether_length.hpp"
#include "marsupial_optimizer/ceres_constraint_tether_parameters.hpp"
#include "marsupial_optimizer/ceres_constraint_parabola_obstacles.hpp"
#include "marsupial_optimizer/ceres_constraint_parabola_length.hpp"
#include "marsupial_optimizer/ceres_constraint_parabola_parameters.hpp"

#include "marsupial_optimizer/marsupial_trajectory_optimized.h"

#include "misc/data_management.hpp"
#include "misc/interpolate_path.hpp"
#include "misc/catenary_solver_ceres.hpp"

#include "catenary_checker/get_tether_parameter.hpp"
#include "catenary_checker/near_neighbor.hpp"
#include "catenary_checker/grid3d.hpp"
#include "catenary_checker/bisection_catenary_3D.h"
#include "catenary_checker/catenary_checker_manager.h"

#include "misc/manage_computed_path.hpp"

//ROS
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

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
#define PRINTF_ORANGE  "\x1B[38;2;255;128;0m"
#define PRINTF_ROSE    "\x1B[38;2;255;151;203m"
#define PRINTF_LBLUE   "\x1B[38;2;53;149;240m"
#define PRINTF_LGREEN  "\x1B[38;2;17;245;120m"
#define PRINTF_GRAY    "\x1B[38;2;176;174;174m"

using namespace Eigen;
using namespace std;

using ceres::AutoDiffCostFunction;
using ceres::NumericDiffCostFunction;
using ceres::CENTRAL;
using ceres::FORWARD;
using ceres::RIDDERS;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
using ceres::LossFunction;
using ceres::CauchyLoss;



struct parameterBlockPos{
	double parameter[4];
};

struct parameterBlockRot{
	double parameter[5];
};

struct parameterBlockTime{
	double parameter[2];
};

struct parameterBlockLength{
	double parameter[2];
};

struct parameterBlockTether{
	double parameter[4];
};

/* Map:
	params[1] = Xo 
	params[2] = Yo 
	params[3] = a 
	params[4] = length (Not used length because it is not optimized)
*/

class TestTetherConstraints
{

public:

	// =========== Function declarations =============
	TestTetherConstraints(std::string node_name_);
	// ~TestTetherConstraints();
	void setupOptimizer();
	void initializeSubscribers();
	void initializePublishers();
	void readOctomapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
	void readPointCloudTraversabilityUGVCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
	void readPointCloudObstaclesUGVCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
	void executeOptimizerPathGoalCB();
	void collisionMapCallBack(const octomap_msgs::OctomapConstPtr &msg);
	void traversableMapCallBack(const octomap_msgs::OctomapConstPtr &msg);
	void finishigOptimization();
	void getReelPose();
	void resetFlags();
	bool CheckStatusTetherCollision(vector<geometry_msgs::Vector3> v1_, vector<geometry_msgs::Vector3 >v2_, vector<tether_parameters> v3_);

	void graphTetherAndPathMarker(vector<geometry_msgs::Vector3> v_ugv_, vector<geometry_msgs::Vector3> v_uav_, 
								   vector<geometry_msgs::Quaternion> v_rot_ugv, vector <tether_parameters> v_p_, vector<float> v_length_,
									int c_ugv_, int c_uav_, int c_tether_, ros::Publisher p_ugv_, ros::Publisher p_uav_, 
									ros::Publisher p_tether_, visualization_msgs::MarkerArray m_);

	// ============= Global Variables ================

	ros::NodeHandlePtr nh;
    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> tf2_list;

	Solver::Options options;
  	Solver::Summary summary;
	float initial_cost, final_cost, successful_steps, unsuccessful_steps, time_optimazation ;
	
	tf::TransformListener listener;
	geometry_msgs::TransformStamped pose_reel_global, pose_reel_local;

	std::string path, files_residuals, name_output_file, user_name, path_mission_file;

    double map_resolution;
	float step , step_inv;
	double ws_x_max, ws_y_max, ws_z_max,ws_x_min,ws_y_min,ws_z_min;
	
	int n_iter_opt;	//Iterations Numbers of Optimizer
	int size_path;
	double distance_tether_obstacle;
	double w_alpha_uav, w_alpha_ugv, w_beta_uav, w_beta_ugv, w_theta_ugv, w_gamma_uav, w_gamma_ugv, w_kappa_ugv, w_kappa_uav, w_delta; 
	double w_epsilon_uav, w_epsilon_ugv, w_zeta_uav , w_zeta_ugv, w_eta_1, w_eta_2, w_eta_3, w_lambda, w_mu_uav, w_nu_ugv;

    bool mapReceivedFull, mapReceivedTrav;
	bool optimize_ugv , optimize_uav, optimize_tether, fix_last_position_ugv, use_distance_function, get_path_from_file;

	NearNeighbor nn_uav; // Kdtree used for Catenary and UAV
	NearNeighbor nn_trav, nn_ugv_obs;
	MarkerPublisher MP;
	Grid3d	*grid_3D, *grid_3D_trav, *grid_3D_obst;
	CatenaryCheckerManager *CheckCM;

	DataManagement dm_;
    sensor_msgs::PointCloud2::ConstPtr pc_obs_ugv;

	//! Manage Data Vertices and Edges
    std::unique_ptr<tf::TransformListener> tf_list;
    std::unique_ptr<tf::TransformListener> tf_list_ptr;
	bool use3d;
    bool debug;
    bool showConfig;
	bool use_catenary_as_tether, print_point_in_graph_marker;

    octomap_msgs::OctomapConstPtr mapFull, mapTrav;
	octomap::OcTree *mapFull_msg , *mapTrav_msg;

	visualization_msgs::MarkerArray points_uav_marker, lines_uav_marker, points_ugv_marker, lines_ugv_marker, 
									post_points_ugv_marker, post_lines_ugv_marker, post_points_uav_marker, post_lines_uav_marker;

	std::string ugv_base_frame, uav_base_frame, reel_base_frame, world_frame;

	ros::Subscriber octomap_ws_sub_,point_cloud_ugv_traversability_sub_, point_cloud_ugv_obstacles_sub_, local_map_sub, local_trav_map_sub, finished_rviz_maneuver_sub_, star_optimizer_process_sub_;
	ros::Publisher traj_marker_ugv_pub_, traj_marker_uav_pub_,traj_opt_marker_ugv_pub_,traj_opt_marker_uav_pub_;

	ros::Publisher  tether_marker_init_pub_, tether_marker_opt_pub_;
	visualization_msgs::MarkerArray catenary_marker, tether_marker_init, tether_marker_opt;

	vector<parameterBlockPos> statesPosUGV;
	vector<parameterBlockPos> statesPosUAV;
	vector<parameterBlockTether> statesTetherParams;
	vector<geometry_msgs::Quaternion> vec_rot_ugv_init, vec_rot_uav_init, vec_rot_ugv_opt, vec_rot_uav_opt;

	vector<double> vec_dist_init_ugv, vec_dist_init_uav;
	vector<double> vec_time_init;
	vector<double> v_angles_smooth_ugv_init, v_angles_smooth_uav_init, v_angles_smooth_ugv_opt, v_angles_smooth_uav_opt;
	vector<float> vec_len_tether_init, vec_len_tether_opt;
	vector<float> vec_cat_param_x0, vec_cat_param_y0, vec_cat_param_a;
	vector<geometry_msgs::Vector3> vec_pose_ugv_opt, vec_pose_uav_opt; 
	vector<geometry_msgs::Vector3> vec_pose_ugv_init, vec_pose_uav_init;
	vector <tether_parameters> v_tether_params_init, v_tether_params_opt;

    double pos_reel_x, pos_reel_y, pos_reel_z;
	ros::Time start_time_opt, final_time_opt;
	int num_pos_initial, num_goal;
	bool write_data_for_analysis, use_loss_function;
	double length_tether_max;
	std::string scenario_name, map_path, map_path_trav, map_path_obst;

	bool equidistance_uav_constraint, obstacles_uav_constraint, smoothness_uav_constraint;
	bool velocity_uav_constraint,acceleration_uav_constraint;
	bool time_constraint, velocity_ugv_constraint, acceleration_ugv_constraint;
	bool tether_length_constraint, tether_obstacle_constraint, tether_parameters_constraint;
	bool finished_rviz_maneuver;
	bool equidistance_ugv_constraint, obstacles_ugv_constraint, traversability_ugv_constraint, smoothness_ugv_constraint;
	bool write_data_residual;

private:
	void cleanVectors();
	double distance_obstacle_uav,distance_obstacle_ugv, initial_velocity_ugv, initial_velocity_uav, angle_min_traj, initial_acceleration_ugv, initial_acceleration_uav;
	double initial_time;
	double min_T;
};

#endif