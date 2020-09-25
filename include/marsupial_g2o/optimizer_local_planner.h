/*
Optimizer Local Planner based in g2o for Marsupial Robotics Configuration 
Simon Martinez Rozas, 2020 
Service Robotics Lab, University Pablo de Olavide , Seville, Spain 
 */


#ifndef _OPTIMIZER_LOCAL_PLANNER_H_
#define _OPTIMIZER_LOCAL_PLANNER_H_

#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/types/icp/types_icp.h"

#include "g2o/stuff/command_args.h"

#include <Eigen/StdVector>
#include <random>
#include <stdint.h>

#include "g2o/solvers/dense/linear_solver_dense.h"

//g2o
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/sparse_optimizer_terminate_action.h>

#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>

#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>

#include <g2o/stuff/macros.h>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

#include "marsupial_g2o/g2o_edge_distance_vertex.h"
#include "marsupial_g2o/g2o_edge_obstacles.h"
#include "marsupial_g2o/g2o_edge_obstacles_through.h"
#include "marsupial_g2o/g2o_edge_distance_xyz.h"
#include "marsupial_g2o/g2o_edge_kinematics.h"
#include "marsupial_g2o/g2o_edge_equi_distance.h"
#include "marsupial_g2o/g2o_edge_time.h"
#include "marsupial_g2o/g2o_edge_velocity.h" 
#include "marsupial_g2o/g2o_edge_acceleration.h" 
#include "marsupial_g2o/g2o_edge_catenary.h"

#include "marsupial_g2o/g2o_vertex_timediff.h"
#include "marsupial_g2o/g2o_vertex_catenary_length.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"

#include "marsupial_g2o/nearNeighbor.hpp"
#include "marsupial_g2o/bisection_catenary_3D.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>

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

// #include <marsupial_actions/OptimizationTrajectoryAction.h>
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

struct structPose
{
	int id;
	Eigen::Vector3d position;
    g2o::VertexPointXYZ *vertex;
};

struct structTime
{
	int id_from;
	int id_to;
	double time;
	// g2o::VertexTimeDiff vertex;
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
	g2o::VertexCatenaryLength *vertex;
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
	void getMarkerPoints(visualization_msgs::MarkerArray &marker_, vector<structPose> _vector, std::string ns, int j_);
	void getMarkerLines(visualization_msgs::MarkerArray &marker_, vector<structPose> _vector, std::string ns, int j_);
	void tfListener(void);
	void configServices();
	void executeOptimizerPathPreemptCB();
	void executeOptimizerPathGoalCB();
	void dynRecCb(marsupial_g2o::OptimizationParamsConfig &config, uint32_t level);
	void collisionMapCallBack(const octomap_msgs::OctomapConstPtr &msg);
	void configMarkers(std::string ns, std::string frame);
	void clearMarkers(auto _s);
	void publishTrajMarker3D(vector<structPose> _vector) ;
	void calculateDistanceVertices(vector<Eigen::Vector3d> _path,vector<structDistance> &_v_sD);
	void getTemporalState(vector<structTime> &_time, vector<structDistance> _v_sD, double _vel);
	void writeTemporalDataBeforeOptimization(void);
	void writeTemporalDataAfterOptimization(auto _s);
	void setInitialLengthCatenaryAndPosUGV(std::vector <double> &_vector, auto _s);
	void getPointsFromGlobalPath(trajectory_msgs::MultiDOFJointTrajectory _path,vector<Eigen::Vector3d> &_v_gp);

	upo_actions::ExecutePathResult action_result;

	// ============= Global Variables ================

	ros::NodeHandlePtr nh;
	tf2_ros::Buffer *tfBuffer;
	
	tf::TransformListener listener;

	ofstream file_in_pos,file_in_time ,file_in_velocity, file_in_difftime, file_in_acceleration;
	ofstream file_out_pos, file_out_time, file_out_velocity, file_out_difftime, file_out_acceleration;
	std::string path= "/home/simon/";
	int n_iter_opt;	//Iterations Numbers of Optimizer
	double initial_multiplicative_factor_length_catenary;
	float radius_collition_catenary,min_distance_add_new_point;
	double w_alpha, w_beta, w_iota, w_gamma, w_delta, w_epsilon, w_zeta, w_eta, w_theta, w_kappa;
	// double initial_pos_ugv_x, initial_pos_ugv_y, initial_pos_ugv_z, initial_pos_ugv_yaw, initial_pos_ugv_pitch, initial_pos_ugv_roll, pos_uav_above_ugv;


	NearNeighbor nn_;

	SparseOptimizer optimizer;
	g2o::VertexPointXYZ* vertexPose;
	g2o::VertexTimeDiff* vertexTime;
	g2o::VertexCatenaryLength* vertexCatenaryLength;

	g2o::G2ODistanceVertexEdge* edgeDistanceVertex;
	g2o::G2ODistanceXYZEdge* edgeDistanceXYZ;
	g2o::G2OKinematicsEdge* edgeKinetics;
	g2o::G2OObstaclesEdge* edgeObstaclesNear;
	g2o::G2OThroughObstaclesEdge* edgeThroughObstacles;
	g2o::G2OEquiDistanceEdge* edgeEquiDistance;
	g2o::G2OCatenaryEdge* edgeCatenary;

	//! Manage Data Vertices and Edges
	vector<float> v_mXYZ_;

	vector<Eigen::Vector3d> new_path_from_global;

	// actionlib::SimpleActionServer<marsupial_actions::OptimizationTrajectoryAction> optimization_trajectory_action_server_;
	// marsupial_actions::OptimizationTrajectoryFeedback feedback_; //variable stores the feedback/intermediate results
    // marsupial_actions::OptimizationTrajectoryResult result_; //variable stores the final output

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
	// std::string uav_base_frame, ugv_base_frame;

	ros::Subscriber octomap_sub_,local_map_sub;
	ros::Publisher traj_marker_pub_,visMarkersPublisher;

	geometry_msgs::Point obs_oct;

	int n_time_optimize;
	bool continue_optimizing;
	double td_;
	bool verbose_optimizer;

	void setStraightTrajectory(double x1, double y1, double z1, double x2, double y2, double z2, int n_v_u_);	//Function to create a straight line from point A to B
	void getSlopXYZAxes( vector<float> &m_);
	typedef vector<structPose> PoseSequence;
	typedef vector<structTime> TimeSequence;
	typedef vector<structDistance> DistanceSequence;
	typedef vector<structLengthCatenary> LengthCatenarySequence;
	PoseSequence pose_vec_opt_; 
	DistanceSequence dist_vec_; 
	TimeSequence time_vec_;
	LengthCatenarySequence len_cat_vec_;
	float d3D_;
	float n_points; 
	vector<float> v_slopeXYZ;
	vector<double> v_initial_length_catenary;

private:
	void resetFlags();
	double global_path_length;
	double bound, velocity , angle_min_traj, acceleration,bound_bisection_a,bound_bisection_b;
	geometry_msgs::Point ugv_pos_catenary;

    // void pointsSub(const PointCloud::ConstPtr &points);

};

#endif