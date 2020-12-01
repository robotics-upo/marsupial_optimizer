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

#include "marsupial_g2o/near_neighbor.hpp"
#include "marsupial_g2o/bisection_catenary_3D.h"
// #include "marsupial_g2o/marker_publisher.hpp"

#include <iostream>
#include <fstream>
#include <vector>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Eigen/Core"

#include "marsupial_g2o/ceres_contrain_equidistance.hpp"
#include "marsupial_g2o/ceres_contrain_obstacles.hpp"
#include "marsupial_g2o/ceres_contrain_kinematics.hpp"
#include "marsupial_g2o/ceres_contrain_time.hpp"
#include "marsupial_g2o/ceres_contrain_velocity.hpp"
#include "marsupial_g2o/ceres_contrain_acceleration.hpp"
#include "marsupial_g2o/ceres_contrain_catenary.hpp"

//ROS
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <octomap_msgs/Octomap.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <upo_actions/ExecutePathAction.h>
#include <upo_actions/NavigateAction.h>
#include <upo_actions/Navigate3DAction.h>
#include "marsupial_g2o/OptimizationParamsConfig.h"

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
using namespace g2o;

using ceres::AutoDiffCostFunction;
using ceres::NumericDiffCostFunction;
using ceres::CENTRAL;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;


struct structTime
{
	int id;
	double time;
};

struct structDistance
{
	int id_from;
	int id_to;
	double dist;
};

struct structLengthCatenary
{
	int id;
	double length;
};

struct parameterBlock{
	double parameter[5];
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
	// void executeOptimization(const marsupial_actions::OptimizationTrajectoryGoalConstPtr &goal);

	void initializeSubscribers();
	void initializePublishers();
	void readOctomapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
	// void getMarkerPoints(visualization_msgs::MarkerArray &marker_, vector<structPose> _vector, std::string ns, int j_);
	// void getMarkerLines(visualization_msgs::MarkerArray &marker_, vector<structPose> _vector, std::string ns, int j_);
	void tfListener(void);
	void configServices();
	void executeOptimizerPathPreemptCB();
	void executeOptimizerPathGoalCB();
	void dynRecCb(marsupial_g2o::OptimizationParamsConfig &config, uint32_t level);
	void collisionMapCallBack(const octomap_msgs::OctomapConstPtr &msg);
	// void configMarkers(std::string ns, std::string frame);
	// void clearMarkers(auto _s);
	void calculateDistanceVertices(vector<Eigen::Vector3d> _path,vector<structDistance> &_v_sD);
	void getTemporalState(vector<structTime> &_time, vector<structDistance> _v_sD, double _vel);
	void writeTemporalDataBeforeOptimization(void);
	void writeTemporalDataAfterOptimization(auto _s);
	void setInitialLengthCatenaryAndPosUGV(std::vector <double> &_vector, auto _s);
	void preComputeLengthCatenary(std::vector <double> &_vector, auto _s);
	void getPointsFromGlobalPath(trajectory_msgs::MultiDOFJointTrajectory _path,vector<Eigen::Vector3d> &_v_gp);
	void checkObstaclesBetweenCatenaries(std::vector <double> _vectorIN,std::vector <double> &_vectorOUT, auto _s);
	void straightTrajectoryVertices(double x1, double y1, double z1, double x2, double y2, double z2, int _n_v_u, std::vector<Eigen::Vector3d> &_v);
	void getDataForOptimizerAnalysis();

	void markerPoints(visualization_msgs::MarkerArray _marker, std::vector<geometry_msgs::Point> _vector, int _suffix, int _n_v, ros::Publisher c_m_pub_);
	void clearMarkers(visualization_msgs::MarkerArray _marker, auto _s, ros::Publisher c_m_pub_);
	void getMarkerPoints(visualization_msgs::MarkerArray &marker_, vector<Eigen::Vector3d> _vector, std::string ns);
	void getMarkerLines(visualization_msgs::MarkerArray &marker_, vector<Eigen::Vector3d> _vector, std::string ns);
	void clearMarkersPointLines(auto _s);

	upo_actions::ExecutePathResult action_result;

	// ============= Global Variables ================

	ros::NodeHandlePtr nh;
	tf2_ros::Buffer *tfBuffer;

	Solver::Options options;
  	Solver::Summary summary;
  	Problem problem;
	
	tf::TransformListener listener;

	ofstream file_in_pos,file_in_time ,file_in_velocity, file_in_difftime, file_in_acceleration;
	ofstream file_out_pos, file_out_time, file_out_velocity, file_out_difftime, file_out_acceleration;
	std::string path;
	int n_iter_opt;	//Iterations Numbers of Optimizer
	double initial_multiplicative_factor_length_catenary;
	double distance_catenary_obstacle,min_distance_add_new_point,dynamic_catenary, initial_distance_states;
	double w_alpha, w_beta, w_gamma, w_delta, w_epsilon, w_zeta, w_eta;

	NearNeighbor nn_;
	MarkerPublisher mp_;

	//! Manage Data Vertices and Edges
	vector<float> v_mXYZ_;

	vector<Eigen::Vector3d> new_path;

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

	visualization_msgs::MarkerArray points_marker, lines_marker;
	typedef visualization_msgs::Marker RVizMarker;

	std::string action_name_;

	ros::Subscriber octomap_sub_,local_map_sub;
	ros::Publisher traj_marker_pub_;

	ros::Publisher catenary_marker_pub_;
	visualization_msgs::MarkerArray catenary_marker;
	// void markerPoints(visualization_msgs::MarkerArray _marker, std::vector<geometry_msgs::Point> _vector, int _suffix, int _n_v);
	// void clearMarkers(visualization_msgs::MarkerArray _marker,auto _s);

	geometry_msgs::Point obs_oct;

	int n_time_optimize;
	bool continue_optimizing;
	double td_;
	bool verbose_optimizer;

	void setStraightTrajectory(double x1, double y1, double z1, double x2, double y2, double z2, int n_v_u_);	//Function to create a straight line from point A to B
	typedef vector<structTime> TimeSequence;
	typedef vector<structDistance> DistanceSequence;
	typedef vector<structLengthCatenary> LengthCatenarySequence;
	vector<parameterBlock> states;
	DistanceSequence vec_dist_init; 
	TimeSequence vec_time_init;
	vector<Eigen::Vector3d> vec_pose_opt; 
	LengthCatenarySequence vec_len_cat_opt , vec_len_cat_init;
	float d3D_;
	float n_points; 
	vector<double> v_pre_initial_length_catenary;
	vector<double> v_initial_length_catenary;

	double z_constraint;

	int count_edges, n_total_edges, n_edges_before_catenary;

	std::ofstream ofs;
	std::string output_file, name_output_file;
	std::vector<double> vec_time_opt, vec_dist_opt, vec_vel_opt, vec_acc_opt;
	std::vector<Eigen::Vector3d> vec_pose_init;
	ros::Time start_time_opt_, final_time_opt_;
	int scenario_number, num_pos_initial, num_goal;
	bool write_data_for_analysis;

private:
	void resetFlags();
	void cleanVectors();
	double global_path_length;
	double distance_obstacle, initial_velocity, angle_min_traj, acceleration,bound_bisection_a,bound_bisection_b;
	geometry_msgs::Point ugv_pos_catenary;

};

#endif