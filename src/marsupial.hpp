#ifndef _MARSUPIAL_HPP_
#define _MARSUPIAL_HPP_

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

#include "marsupial_g2o/g2o_vertex_timediff.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"

// #include "marsupial_g2o/obstacles.hpp"
#include "marsupial_g2o/nearNeighbor.hpp"
#include "marsupial_g2o/marsupial_cfg.hpp"

#include <iostream>
#include <fstream>
#include <vector>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>

//ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <marsupial_actions/OptimizationTrajectoryAction.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>


using namespace Eigen;
using namespace std;
using namespace g2o;

class Marsupial
{

public:

	// ============= Global Variables ================
	ofstream file_in_pos,file_in_time ,file_in_velocity, file_in_difftime, file_in_acceleration;
	ofstream file_out_pos, file_out_time, file_out_velocity, file_out_difftime, file_out_acceleration;
	string path= "/home/simon/";
	double X1,Y1,Z1,X2,Y2,Z2;
	int n_iter_opt;	//Iterations Numbers of Optimizer
	int n_vert_unit; //number of Vertex per unit of lenght
	double w_alpha, w_beta, w_iota, w_gamma, w_delta, w_epsilon, w_zeta, w_eta, w_theta;

	double bound, velocity , angle, acceleration;

	MarsupialCfg mcfg;

	NearNeighbor nn_;

	SparseOptimizer optimizer;
	g2o::VertexPointXYZ* vertexPose;
	g2o::VertexTimeDiff* vertexTime;

	g2o::G2ODistanceVertexEdge* edgeDistanceVertex;
	g2o::G2OObstaclesEdge* edgeObstaclesNear;
	g2o::G2ODistanceXYZEdge* edgeDistanceXYZ;
	g2o::G2OKinematicsEdge* edgeKinetics;
	g2o::G2OEquiDistanceEdge* edgeEquiDistance;

	//! Manage Data Vertices and Edges
	// vector<Eigen::Vector3d> vecObs;
	vector<Eigen::Vector3d> vecOpt;
	vector<Eigen::Vector3d> near_obs;
	vector<float> mXYZ_;
	//! Create obstacles object
	// Obstacles _Obs1;
	// Obstacles _Obs2;
	// Obstacles _Obs3;

	actionlib::SimpleActionServer<marsupial_actions::OptimizationTrajectoryAction> optimization_trajectory_action_server_;
	marsupial_actions::OptimizationTrajectoryFeedback feedback_; //variable stores the feedback/intermediate results
    marsupial_actions::OptimizationTrajectoryResult result_; //variable stores the final output

	// bool *finish;
	// bool aux;

	std::string action_name_;

	ros::Subscriber read_octomap_sub_;
	ros::Publisher init_traj_pub_,final_traj_pub_, line_traj_pub_;

	geometry_msgs::Point obs_oct;

	int n_time_optimize;
	bool continue_optimizing;
	double td_;

	// =========== Function declarations =============
	Marsupial(ros::NodeHandle &nh, ros::NodeHandle &pnh, std::string name);
	// ~Marsupial();
	void setupOptimization();
	void executeOptimization(const marsupial_actions::OptimizationTrajectoryGoalConstPtr &goal);

	void initializeSubscribers(ros::NodeHandle &nh);
	void initializePublishers(ros::NodeHandle &nh);
	void readOctomapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
	void getMarkerArray(visualization_msgs::MarkerArray &_marker, Eigen::Vector3d _p);

};

Marsupial::Marsupial(ros::NodeHandle &nh, ros::NodeHandle &pnh,std::string name):
			mcfg(nh,pnh),
			optimization_trajectory_action_server_(name,boost::bind(&Marsupial::executeOptimization, this,_1),false),
			action_name_(name){
	
	pnh.param<double>("X1", X1, 25.0);
  	pnh.param<double>("Y1", Y1, 40.0);
	pnh.param<double>("Z1", Z1, 50.0);
	pnh.param<double>("X2", X2, 50.0);
  	pnh.param<double>("Y2", Y2, 15.0);
  	pnh.param<double>("Z2", Z2, 40.0);
  	
	pnh.param<double>("w_alpha", w_alpha,0.1);
	pnh.param<double>("w_beta",	w_beta,0.1);
	pnh.param<double>("w_iota",	w_iota,0.1);
  	pnh.param<double>("w_gamma", w_gamma,0.1);
  	pnh.param<double>("w_delta", w_delta,0.1);
  	pnh.param<double>("w_epsilon", w_epsilon,0.1);
  	pnh.param<double>("w_zeta", w_zeta,0.1);
  	pnh.param<double>("w_eta", w_eta,0.1);
  	pnh.param<double>("w_theta", w_theta,0.1);

	pnh.param<int>("n_iter_opt", n_iter_opt,200);
  	pnh.param<int>("n_vert_unit", n_vert_unit,4);
  	pnh.param<double>("bound", bound,2.0);
  	pnh.param<double>("velocity", velocity,1.0);
  	pnh.param<double>("acceleration", acceleration,0.0);
  	pnh.param<double>("angle", angle, M_PI / 15.0);
	
  	pnh.param<int>("n_time_optimize", n_time_optimize, 500);
  	pnh.param<double>("td_", td_, 0.5);

	continue_optimizing = false;
	
	
	initializeSubscribers(nh);
	initializePublishers(nh);

	setupOptimization();
	optimization_trajectory_action_server_.start();

	ROS_INFO("Parameters loaded, ready to Initialize Optimization Proccess !!");
}

void Marsupial::initializeSubscribers(ros::NodeHandle &nh)
{
    read_octomap_sub_ = nh.subscribe( "octomap_point_cloud_centers", 2,  &Marsupial::readOctomapCallback, this);
    ROS_INFO("Subscribers Initialized marsupial_node");
}

void Marsupial::initializePublishers(ros::NodeHandle &nh)
{
    init_traj_pub_ = nh.advertise<visualization_msgs::MarkerArray>("initial_trajectory", 2);
    final_traj_pub_ = nh.advertise<visualization_msgs::MarkerArray>("final_trajectory", 2);
    line_traj_pub_ = nh.advertise<visualization_msgs::MarkerArray>("line_trajectory", 2);
    ROS_INFO("Publishers Initialized marsupial_node");
}

void Marsupial::readOctomapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	  nn_.setInput(*msg);
	//   printf("1  nn_.obs_points=[%lu]",nn_.obs_points->size());
}

void Marsupial::setupOptimization()
{
	mcfg.setNumVertUnit(n_vert_unit);	
	mcfg.setVelocity(velocity);
	// mcfg.setDistance3D(X1,Y1,Z1,X2,Y2,Z2);
	// mcfg.setNumPoints(mcfg.d3D_,mcfg.n_vert_unit);

	//! Create the linear solver (Ax = b)
	auto linearSolver = g2o::make_unique<LinearSolverCSparse<BlockSolverX::PoseMatrixType>>();
    // auto linearSolver = g2o::make_unique<LinearSolverDense<BlockSolverX::PoseMatrixType>>(); // Linear equation solver

	auto blockSolver = g2o::make_unique<BlockSolverX>(move(linearSolver));
	
	OptimizationAlgorithmLevenberg* optimizationAlgorithm = new OptimizationAlgorithmLevenberg(move(blockSolver));
	// OptimizationAlgorithmGaussNewton* optimizationAlgorithm = new g2o::OptimizationAlgorithmGaussNewton(move(blockSolver));
    // OptimizationAlgorithmDogleg* optimizationAlgorithm = new OptimizationAlgorithmDogleg(move(blockSolver));

	// optimizationAlgorithm->setMaxTrialsAfterFailure(3);
	optimizationAlgorithm->setUserLambdaInit(0.00001);

	optimizer.clear();
	optimizer.setAlgorithm(optimizationAlgorithm);
    optimizer.setVerbose(true);
	
	//! Create the vectors with initial poses and distances between vertex
	ROS_INFO("Loading Trajectory Points");
	mcfg.setStraightTrajectory(X1,Y1,Z1,X2,Y2,Z2,n_vert_unit);
	ROS_INFO("X1=[%f] Y1=[%f] Z1=[%f] X2=[%f] Y2=[%f] Z2=[%f]\n",X1,Y1,Z1,X2,Y2,Z2);
	ROS_INFO("alpha=[%f] beta=[%f] gamma=[%f] delta=[%f] epsilon=[%f] zeta=[%f] eta=[%f] theta=[%f]",w_alpha,w_beta,w_gamma,w_delta,w_epsilon,w_zeta,w_eta,w_theta);
	
	//! Create Objet for kdtree
	mcfg.getSlopXYZAxes(mXYZ_);
}


void Marsupial::executeOptimization(const marsupial_actions::OptimizationTrajectoryGoalConstPtr &goal)
{

	for (int j_ = 0; j_ < n_time_optimize; j_++)
	{
		if (!continue_optimizing)
			printf("Preparing to execute optimization!!\n");
			// printf("Preparing to execute optimization [%i]!!\n",j_);
		// delete vertexPose;
		vertexPose = new g2o::VertexPointXYZ;
		vertexTime = new g2o::VertexTimeDiff;

		visualization_msgs::MarkerArray initial_marker, final_marker , line_strip;
		initial_marker.markers.clear();
		final_marker.markers.clear();
		line_strip.markers.clear();
		initial_marker.markers.resize(mcfg.pose_vec_.size());
		final_marker.markers.resize(mcfg.pose_vec_.size());
		line_strip.markers.resize(mcfg.pose_vec_.size());

		/*
		*Create Vertex. Points of the trajectory 
		*/
		//! To save points cable before and after optimization
		for (size_t i = 0; i < mcfg.pose_vec_.size(); ++i)
		{	
			vertexPose = new g2o::VertexPointXYZ();
			Eigen::Vector3d xyz;
			if(!continue_optimizing){
				xyz = mcfg.pose_vec_[i].position;
				initial_marker.markers[i].header.frame_id = "/map";
				initial_marker.markers[i].header.stamp = ros::Time::now();
				initial_marker.markers[i].ns = "points";
				initial_marker.markers[i].id = i;
				// initial_marker.markers[i].action = visualization_msgs::Marker::DELETEALL;
				initial_marker.markers[i].type = visualization_msgs::Marker::SPHERE;
				initial_marker.markers[i].lifetime = ros::Duration(0);
				initial_marker.markers[i].pose.position.x = xyz.x();
				initial_marker.markers[i].pose.position.y = xyz.y();
				initial_marker.markers[i].pose.position.z = xyz.z();
				initial_marker.markers[i].pose.orientation.x = 0.0;
				initial_marker.markers[i].pose.orientation.y = 0.0;
				initial_marker.markers[i].pose.orientation.z = 0.0;
				initial_marker.markers[i].pose.orientation.w = 1.0;
				initial_marker.markers[i].scale.x = 0.1;
				initial_marker.markers[i].scale.y = 0.1;
				initial_marker.markers[i].scale.z = 0.1;
				initial_marker.markers[i].color.a = 1.0;
				initial_marker.markers[i].color.r = 0.0;
				initial_marker.markers[i].color.g = 0.9;
				initial_marker.markers[i].color.b = 0.0;

				vertexPose->setId(mcfg.pose_vec_[i].id);
				vertexPose->setEstimate(xyz);
				//First and last vertex are fixed. 	
				if (i == 0 || i == mcfg.pose_vec_.size()-1)
					vertexPose->setFixed(true);
			}
			else{
				vertexPose->setId(mcfg.pose_vec_[i].id);
				vertexPose->setEstimate(vecOpt[i]);
					//First and last vertex are fixed. 	
				if (i == 0 || i == mcfg.pose_vec_.size()-1)
					vertexPose->setFixed(true);
			}
			optimizer.addVertex(vertexPose);	
			

		}
		if (!continue_optimizing)
			init_traj_pub_.publish(initial_marker);


		//! I. Add edges. These are the distances between the poses.
		for (size_t i = 0; i < mcfg.dist_vec_.size(); ++i)
		{
			edgeDistanceVertex = new g2o::G2ODistanceVertexEdge();
			edgeDistanceVertex->vertices()[0] = optimizer.vertices()[mcfg.dist_vec_[i].id_from];
			edgeDistanceVertex->vertices()[1] = optimizer.vertices()[mcfg.dist_vec_[i].id_to];
			edgeDistanceVertex->setMeasurement(mcfg.d3D_/mcfg.n_points);
			edgeDistanceVertex->setInformation(G2ODistanceVertexEdge::InformationType::Identity()*w_alpha); 
			optimizer.addEdge(edgeDistanceVertex);
		}
		//! II. Add edges. These are the XYZ distances between Vertices.
		for (size_t i = 0; i < mcfg.dist_vec_.size(); ++i)
		{
			edgeDistanceXYZ = new g2o::G2ODistanceXYZEdge();
			edgeDistanceXYZ->vertices()[0] = optimizer.vertices()[mcfg.dist_vec_[i].id_from];
			edgeDistanceXYZ->vertices()[1] = optimizer.vertices()[mcfg.dist_vec_[i].id_to];
			edgeDistanceXYZ->setMeasurement(mXYZ_);
			edgeDistanceXYZ->setInformation(G2ODistanceXYZEdge::InformationType::Identity()*w_gamma); 
			optimizer.addEdge(edgeDistanceXYZ);
		}
		// ! III. Add edges. This is Equi-distance among vertices.
		for (size_t i = 0; i < mcfg.pose_vec_.size()-3; ++i)
		{
			edgeEquiDistance = new g2o::G2OEquiDistanceEdge;
			edgeEquiDistance->vertices()[0] = optimizer.vertices()[i];
			edgeEquiDistance->vertices()[1] = optimizer.vertices()[i+1];
			edgeEquiDistance->vertices()[2] = optimizer.vertices()[i+2];
			edgeEquiDistance->setInformation(G2OEquiDistanceEdge::InformationType::Identity()*w_epsilon);
			optimizer.addEdge(edgeEquiDistance);
		}
		//! IV. Add edges. These are the distances from vertex to obstacles poses.
		for (size_t i = 0; i < mcfg.pose_vec_.size(); ++i)
		{
			edgeObstaclesNear = new g2o::G2OObstaclesEdge();
			edgeObstaclesNear->vertices()[0] = optimizer.vertices()[i];
			edgeObstaclesNear->readKDTree(nn_.kdtree,nn_.obs_points);
			edgeObstaclesNear->setMeasurement(bound);
			edgeObstaclesNear->setInformation(G2OObstaclesEdge::InformationType::Identity()*w_beta);
			optimizer.addEdge(edgeObstaclesNear);
		}
		//! V. Add edges. These are the distances from vertex to obstacles poses.
		for (size_t i = 0; i < mcfg.dist_vec_.size(); ++i)
		{
			g2o::G2OThroughObstaclesEdge* edge = new g2o::G2OThroughObstaclesEdge();
			edge->vertices()[0] = optimizer.vertices()[mcfg.dist_vec_[i].id_from];
			edge->vertices()[1] = optimizer.vertices()[mcfg.dist_vec_[i].id_to];
			edge->setRobustKernel(new g2o::RobustKernelCauchy());
            edge->robustKernel()->setDelta(1.0);
			edge->readKDTree(nn_.kdtree,nn_.obs_points);
			edge->setInformation(G2OThroughObstaclesEdge::InformationType::Identity()*w_iota); 
			optimizer.addEdge(edge);
		}
		// VI. Add edges. This is kinematic of trajectory.
		for (size_t i = 0; i < mcfg.pose_vec_.size()-3; ++i)
		{
			edgeKinetics = new g2o::G2OKinematicsEdge;
			edgeKinetics->vertices()[0] = optimizer.vertices()[i];
			edgeKinetics->vertices()[1] = optimizer.vertices()[i+1];
			edgeKinetics->vertices()[2] = optimizer.vertices()[i+2];
			edgeKinetics->setMeasurement(angle);
			edgeKinetics->setInformation(G2OKinematicsEdge::InformationType::Identity()*w_delta);
			optimizer.addEdge(edgeKinetics);
		}
		/*
		*Create Vertex. Time between consecutive configuration 
		*/
		//! To save temporal state before optimization
		// file_in_time.open ((path+"initial_time.txt"));
		// file_in_velocity.open ((path+"initial_velocity.txt"));
		// file_in_difftime.open ((path+"initial_difftime.txt"));
		// file_in_acceleration.open ((path+"initial_acceleration.txt"));
		// double _dT = 0.0; 
		// double _sum_dT = 0.0;
		// double _sum_dist = 0.0;
		// for (size_t i = 0; i < mcfg.dist_vec_.size(); ++i)
		// {	
		// 	_dT = mcfg.time_vec_[i].time;
		// 	_sum_dT = _dT + _sum_dT;
		// 	_sum_dist = mcfg.dist_vec_[i].dist + _sum_dist;
		// 	vertexPose = new g2o::VertexTimeDiff();
		// 	if (i == 0)	//First vertex is fixed. 
		// 		vertexTime->setFixed(true);
		// 		// vertexTime = new g2o::VertexTimeDiff(_dT,true);
		// 	else
		// 		vertexTime->setFixed(false);
		// 		// vertexTime = new g2o::VertexTimeDiff(_dT,false);
		// 	vertexTime->setId(i+mcfg.dist_vec_.size()+1);
		// 	vertexTime->setEstimate(_dT);
		// 	optimizer.addVertex(vertexTime);	
		// 	file_in_time << setprecision(6) << _sum_dist << ";" << _sum_dT << endl;
		// 	file_in_velocity  << setprecision(6) << _sum_dist << ";" << velocity << endl;
		// 	file_in_difftime << setprecision(6) << _sum_dist << ";" << _dT << endl;
		// 	if (i > 0)
		// 		file_in_acceleration << setprecision(6) << _sum_dist << ";" << acceleration << endl;
		// }
		// file_in_time.close();	
		// file_in_velocity.close();	
		// file_in_difftime.close();	
		// file_in_acceleration.close();


		//! VI. Add edges. These are time between Vertices.
		// for (unsigned i=mcfg.dist_vec_.size()+1; i < mcfg.dist_vec_.size()*2.0; ++i)
		// {
		// 	G2OTimeEdge* edge = new G2OTimeEdge;
		// 	edge->vertices()[0] = optimizer.vertices()[i];
		// 	edge->setMeasurement(mcfg.time_vec_[i-mcfg.dist_vec_.size()-1].time);
		// 	edge->setInformation(G2OTimeEdge::InformationType::Identity()*w_zeta);
		// 	optimizer.addEdge(edge);
		// }

		//! VII. Add edges. These are velocities between Vertices.
		// for (size_t i = 0; i < mcfg.dist_vec_.size(); ++i)
		// {
		// 	G2OVelocityEdge* edge = new G2OVelocityEdge;
		// 	edge->vertices()[0] = optimizer.vertices()[mcfg.dist_vec_[i].id_from];
		// 	edge->vertices()[1] = optimizer.vertices()[mcfg.dist_vec_[i].id_to];
		// 	edge->vertices()[2] = optimizer.vertices()[i+mcfg.dist_vec_.size()+1];
		// 	edge->setMeasurement(velocity);
		// 	edge->setInformation(G2OVelocityEdge::InformationType::Identity()*w_eta);
		// 	optimizer.addEdge(edge);
		// }

		//! VIII. Add edges. This is aceleration between Vertices.
		// for (size_t i = 0; i < mcfg.dist_vec_.size()-1; ++i)
		// {
		// 	G2OAccelerationEdge* edge = new G2OAccelerationEdge;
		// 	edge->vertices()[0] = optimizer.vertices()[i];
		// 	edge->vertices()[1] = optimizer.vertices()[i+1];
		// 	edge->vertices()[2] = optimizer.vertices()[i+2];
		// 	edge->vertices()[3] = optimizer.vertices()[i+mcfg.dist_vec_.size()+1];
		// 	edge->vertices()[4] = optimizer.vertices()[i+mcfg.dist_vec_.size()+2];
		// 	edge->setMeasurement(acceleration);
		// 	edge->setInformation(G2OAccelerationEdge::InformationType::Identity()*w_theta);
		// 	optimizer.addEdge(edge);
		// }

		if (!continue_optimizing)
			std::cout << "Optimization  started !!" << std::endl;
		// optimizer.save("antithing_before.g2o");
		optimizer.initializeOptimization();
		optimizer.setVerbose(true);
		optimizer.optimize(n_iter_opt);
		// optimizer.save("antithing_after.g2o");

		//! Save Optimized Trajectory in File.txt 
		// file_out_pos.open (path+"optimized_trajectory.txt");
		for (unsigned i = 0; i < mcfg.pose_vec_.size(); i++)
		{
			structPose vOp;
			g2o::VertexPointXYZ *pose = dynamic_cast<g2o::VertexPointXYZ*>(optimizer.vertex(i));
			vOp.position = pose->estimate();	
			vOp.id = i;
			// file_out_pos << setprecision(6) << vOp.position.x() << ";" << vOp.position.y() << ";" << vOp.position.z()<< endl;

			final_marker.markers[i].header.frame_id = "/map";
			final_marker.markers[i].header.stamp = ros::Time::now();
			final_marker.markers[i].ns = "points";
			final_marker.markers[i].id = i;
			// final_marker.markers[i].action = visualization_msgs::Marker::ADD;
			if (i % 5 == 0)
				final_marker.markers[i].type = visualization_msgs::Marker::CUBE;
			else
				final_marker.markers[i].type = visualization_msgs::Marker::SPHERE;
			

			if (j_+1 == n_time_optimize)
				final_marker.markers[i].lifetime = ros::Duration(0);
			else
				final_marker.markers[i].lifetime = ros::Duration(2.0);
			final_marker.markers[i].pose.position.x = vOp.position.x();
			final_marker.markers[i].pose.position.y = vOp.position.y();
			final_marker.markers[i].pose.position.z = vOp.position.z();
			final_marker.markers[i].pose.orientation.x = 0.0;
			final_marker.markers[i].pose.orientation.y = 0.0;
			final_marker.markers[i].pose.orientation.z = 0.0;
			final_marker.markers[i].pose.orientation.w = 1.0;
			final_marker.markers[i].scale.x = 0.1;
			final_marker.markers[i].scale.y = 0.1;
			final_marker.markers[i].scale.z = 0.1;
			final_marker.markers[i].color.a = 1.0;
			final_marker.markers[i].color.r = 0.0;
			final_marker.markers[i].color.g = 0.0;
			final_marker.markers[i].color.b = 0.9;

			// geometry_msgs::Point _p1, _p2;
			// if (i > 0)
			// {
			// 	_p2.x= vOp.position.x();
			// 	_p2.y= vOp.position.y();
			// 	_p2.z= vOp.position.z();
			// 	line_strip.markers[i].header.frame_id = "/map";
			// 	line_strip.markers[i].header.stamp = ros::Time::now();
			// 	line_strip.markers[i].ns = "points";
			// 	line_strip.markers[i].id = i;
			// 	line_strip.markers[i].type = visualization_msgs::Marker::LINE_STRIP;
			// 	if (j_+1 == n_time_optimize)
			// 		line_strip.markers[i].lifetime = ros::Duration(0);
			// 	else
			// 		line_strip.markers[i].lifetime = ros::Duration(1.0);		
			// 	line_strip.markers[i].points.push_back(_p1);
			// 	line_strip.markers[i].points.push_back(_p2);
			// 	line_strip.markers[i].pose.orientation.x = 0.0;
			// 	line_strip.markers[i].pose.orientation.y = 0.0;
			// 	line_strip.markers[i].pose.orientation.z = 0.0;
			// 	line_strip.markers[i].pose.orientation.w = 1.0;
			// 	line_strip.markers[i].scale.x = 0.02;
			// 	line_strip.markers[i].color.a = 1.0;
			// 	line_strip.markers[i].color.r = 0.0;
			// 	line_strip.markers[i].color.g = 0.0;
			// 	line_strip.markers[i].color.b = 0.9;
			// }
			// _p1.x= vOp.position.x();
			// _p1.y= vOp.position.y();
			// _p1.z= vOp.position.z();
		}
		// file_out_pos.close();
		final_traj_pub_.publish(final_marker);
		line_traj_pub_.publish(line_strip);

		vecOpt.clear();
		for (size_t i = 0; i < mcfg.pose_vec_.size(); i++)
		{
			g2o::VertexPointXYZ *pose = dynamic_cast<g2o::VertexPointXYZ*>(optimizer.vertex(i));
			vecOpt.push_back(pose->estimate());

			optimizer.removeVertex(optimizer.vertex(i));
		}
		continue_optimizing = true;
		

		//! Save Optimized Time in File.txt 
		// file_out_time.open (path+"optimized_time.txt");
		// file_out_velocity.open (path+"optimized_velocity.txt");
		// file_out_difftime.open (path+"optimized_difftime.txt");
		// double sum_dispos = 0.0;
		// double sum_diffTime = 0.0;
		// double velocity = 0.0;
		// for (size_t i = 0; i < mcfg.dist_vec_.size(); i++)
		// {
		// 	g2o::VertexPointXYZ *pose1Op = dynamic_cast<g2o::VertexPointXYZ*>(optimizer.vertex(i));
		// 	g2o::VertexPointXYZ *pose2Op = dynamic_cast<g2o::VertexPointXYZ*>(optimizer.vertex(i+1));
		// 	g2o::VertexTimeDiff *difftime = dynamic_cast<g2o::VertexTimeDiff*>(optimizer.vertex(i+mcfg.dist_vec_.size()+1));
		// 	double distance = (pose2Op->estimate()-pose1Op->estimate()).norm();	
		// 	sum_dispos = distance + sum_dispos;
		// 	sum_diffTime = difftime->estimate()+ sum_diffTime;
		// 	velocity = distance / difftime->estimate();
		// 	file_out_time << setprecision(6) << sum_dispos << ";" << sum_diffTime << endl;
		// 	file_out_velocity << setprecision(6) << sum_dispos << ";" << velocity << endl;
		// 	file_out_difftime << setprecision(6) << sum_dispos << ";" << difftime->estimate() << endl;
		// 	// printf("p1=[%f %f %f]  p2=[%f %f %f] d=%f Sum_d=%f\n",pose1Op->estimate().x(),pose1Op->estimate().y(),pose1Op->estimate().z(),pose2Op->estimate().x(),pose2Op->estimate().y(),pose2Op->estimate().z(),distance,sum_dispos);
		// }
		// file_out_time.close();
		// file_out_velocity.close();
		// file_out_difftime.close();

		// file_out_acceleration.open (path+"optimized_acceleration.txt");
		// double sumdis = 0.0;
		// for (size_t i = 0; i < mcfg.dist_vec_.size()-1; i++)
		// {
		// 	g2o::VertexPointXYZ *pose1Op = dynamic_cast<g2o::VertexPointXYZ*>(optimizer.vertex(i));
		// 	g2o::VertexPointXYZ *pose2Op = dynamic_cast<g2o::VertexPointXYZ*>(optimizer.vertex(i+1));
		// 	g2o::VertexPointXYZ *pose3Op = dynamic_cast<g2o::VertexPointXYZ*>(optimizer.vertex(i+2));
		// 	g2o::VertexTimeDiff *difftime1 = dynamic_cast<g2o::VertexTimeDiff*>(optimizer.vertex(i+mcfg.dist_vec_.size()+1));
		// 	g2o::VertexTimeDiff *difftime2 = dynamic_cast<g2o::VertexTimeDiff*>(optimizer.vertex(i+mcfg.dist_vec_.size()+1));
			
		// 	double distance1 = (pose2Op->estimate()-pose1Op->estimate()).norm();	
		// 	double distance2 = (pose3Op->estimate()-pose2Op->estimate()).norm();
		// 	if (i==0)
		// 		sumdis = distance1;
		// 	sumdis = sumdis + distance2;
		// 	double diffTime = difftime1->estimate()+ difftime2->estimate();
		// 	double velocity1 = distance1 / difftime1->estimate();
		// 	double velocity2 = distance2 / difftime2->estimate();
		// 	double acceleration = (velocity2-velocity1)/diffTime;
		// 	file_out_acceleration << setprecision(6) << sumdis << ";" << acceleration << endl;
		// }
		// file_out_acceleration.close();

		ros::Duration(td_).sleep();

	}
	if (optimization_trajectory_action_server_.isPreemptRequested() || !ros::ok())
    {
        optimizer.clear();
        ROS_INFO("%s: Preempted", action_name_.c_str());
        result_.optimization_completed = false;
        optimization_trajectory_action_server_.setPreempted(result_);
    }

	std::cout << "Optimization Proccess  Completed !!!" << std::endl;
	result_.optimization_completed = true;
	ROS_INFO("%s: Succeeded", action_name_.c_str());
	optimization_trajectory_action_server_.setSucceeded(result_);
	optimizer.clear();

}

void Marsupial::getMarkerArray(visualization_msgs::MarkerArray &_marker, Eigen::Vector3d _p)
{

	for (size_t i = 0; i < mcfg.pose_vec_.size(); ++i)
	{	
		_marker.markers[i].header.frame_id = "map";
		_marker.markers[i].header.stamp = ros::Time::now();
		_marker.markers[i].ns = "points";
		_marker.markers[i].id = i;
		// _marker.markers[i].action = visualization_msgs::Marker::ADD;
		_marker.markers[i].type = visualization_msgs::Marker::SPHERE;
		_marker.markers[i].lifetime = ros::Duration(0);
		_marker.markers[i].pose.position.x = _p.x();
		_marker.markers[i].pose.position.y = _p.y();
		_marker.markers[i].pose.position.z = _p.z();
		_marker.markers[i].pose.orientation.x = 0.0;
		_marker.markers[i].pose.orientation.y = 0.0;
		_marker.markers[i].pose.orientation.z = 0.0;
		_marker.markers[i].pose.orientation.w = 1.0;
		_marker.markers[i].scale.x = 0.1;
		_marker.markers[i].scale.y = 0.1;
		_marker.markers[i].scale.z = 0.1;
		_marker.markers[i].color.a = 1.0;
		_marker.markers[i].color.r = 0.0;
		_marker.markers[i].color.g = 0.9;
		_marker.markers[i].color.b = 0.0;
	}

}

#endif