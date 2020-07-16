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
#include "marsupial_g2o/g2o_edge_distance_xyz.h"
#include "marsupial_g2o/g2o_edge_kinematics.h"
#include "marsupial_g2o/g2o_edge_equi_distance.h"
#include "marsupial_g2o/g2o_edge_time.h"
#include "marsupial_g2o/g2o_edge_velocity.h" 

// #include "g2o_vertex_timediff.h"
// #include "g2o_vertex_pointxyz.h"

#include "marsupial_g2o/obstacles.hpp"
#include "marsupial_g2o/nearNeighbor.hpp"
#include "marsupial_g2o/marsupial_cfg.hpp"

#include <iostream>
#include <fstream>
#include <vector>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>

#include "ros/ros.h"


using namespace Eigen;
using namespace std;
using namespace g2o;

class Marsupial
{

public:

	// ============= Global Variables ================
	ofstream file_in_pos,file_in_time ,file_in_velocity, file_in_difftime;
	ofstream file_out_pos, file_out_time, file_out_velocity, file_out_difftime;
	string path= "/home/simon/";
	double X1,Y1,Z1,X2,Y2,Z2;
	int n_inter_opt;	//Iterations Numbers of Optimizer
	int n_inter;	//Number of times that optimizer is execute
	int n_vert_unit; //number of Vertex per unit of lenght
	double w_alpha, w_beta, w_gamma, w_delta, w_epsilon, w_zeta, w_eta;
	// double w_eta = 0.1;
	// double w_theta = 0.1;
	double bound, velocity , angle;

	g2o::VertexXYZ* vertex1;
	g2o::VertexTimeDiff* vertex2;

	MarsupialCfg mcfg;

	// =========== Function declarations =============
	Marsupial(ros::NodeHandle &nh, ros::NodeHandle &pnh);
	// ~Marsupial();
	void executeOptimization();

	// =============== Main function =================
};

Marsupial::Marsupial(ros::NodeHandle &nh, ros::NodeHandle &pnh):mcfg(nh,pnh){

	ROS_INFO("Initialazing Optimization Proccess");
	
	if (!pnh.getParam("X1", X1)) { 
    	X1 = 25;
  	}
  	if (!pnh.getParam("Y1", Y1)) {
  		Y1 = 40;
  	}
	if (!pnh.getParam("Z1", Z1)) {
  	  	Z1 = 50;
  	}
	if (!pnh.getParam("X2", X2)) {
  		X2 = 50;
  	}
  	if (!pnh.getParam("Y2", Y2)) {
  		Y2 = 15;
  	}
  	if (!pnh.getParam("Z2", Z2)) {
  	  	Z2 = 40;
  	}

  	if (!pnh.getParam("w_alpha", w_alpha)) {
  	  w_alpha = 0.1;
  	}
  	if (!pnh.getParam("w_beta", w_beta)) {
  	  w_beta = 0.1;
  	}
  	if (!pnh.getParam("w_gamma", w_gamma)) {
  	  w_gamma = 0.1;
  	}
  	if (!pnh.getParam("w_delta", w_delta)) {
  	  w_delta = 0.1;
  	}
  	if (!pnh.getParam("w_epsilon", w_epsilon)) {
  	  w_epsilon = 0.1;
  	}
  	if (!pnh.getParam("w_zeta", w_zeta)) {
  	  w_zeta = 0.1;
  	}
  	if (!pnh.getParam("w_eta", w_eta)) {
  	  w_eta = 0.1;
  	}
  
//   ROS_INFO("n_iter = %d \t K: %d \t Cost weight: %f", n_iter, K, cost_weight);

	executeOptimization();
}

void Marsupial::executeOptimization()
{

	// X1 =25; Y1 =40; Z1 =50;
	// X2 =50; Y2 =15; Z2 =40;

	n_inter_opt = 200;
	n_inter = 1;
	n_vert_unit = 4;

	bound = 2.0;
	velocity = 1.0;
	angle = M_PI / 15.0;
	// deltaTime = (mcfg.d3D_/mcfg.n_points * 1.0/vel);

	//Weight Factors for position edges
	// w_alpha = 0.3; //distanceVertex
	// w_beta = 1.0; //obstacles
	// w_gamma= 0.3; //distanceXYZ
	// w_delta= 0.8; //kinetics
	// w_eta = 0.4;	// equi-distance
	// //Weight Factors for temporal edges
	// w_epsilon = 0.00018;
	// w_zeta = 0.00018;
	// w_eta = 0.1;
	// w_theta = 0.1;

	mcfg.setNumVertUnit(n_vert_unit);	
	mcfg.setDistance3D(X1,Y1,Z1,X2,Y2,Z2);
	mcfg.setNumPoints(mcfg.d3D_,mcfg.n_vert_unit);
	mcfg.setVelocity(velocity);


	//Create the linear solver (Ax = b)
	auto linearSolver = g2o::make_unique<LinearSolverCSparse<BlockSolverX::PoseMatrixType>>();
    // auto linearSolver = g2o::make_unique<LinearSolverDense<BlockSolverX::PoseMatrixType>>(); // Linear equation solver

	auto blockSolver = g2o::make_unique<BlockSolverX>(move(linearSolver));
	
	OptimizationAlgorithmLevenberg* optimizationAlgorithm = new OptimizationAlgorithmLevenberg(move(blockSolver));
	// OptimizationAlgorithmGaussNewton* optimizationAlgorithm = new g2o::OptimizationAlgorithmGaussNewton(move(blockSolver));
    // OptimizationAlgorithmDogleg* optimizationAlgorithm = new OptimizationAlgorithmDogleg(move(blockSolver));

	// optimizationAlgorithm->setMaxTrialsAfterFailure(3);
	optimizationAlgorithm->setUserLambdaInit(0.00001);

	SparseOptimizer optimizer;
	optimizer.setAlgorithm(optimizationAlgorithm);
    optimizer.setVerbose(true);

	//Data
	vector<Eigen::Vector3d> vecObs;
	vector<Eigen::Vector3d> near_obs;
	vector<float> mXYZ_;
	
	//Create obstacles
	Obstacles _Obs1;
	Obstacles _Obs2;
	Obstacles _Obs3;
	_Obs1.simulateObstacles(35, 0, 36,10, 40, 8,vecObs,1);
	_Obs2.simulateObstacles(37, 0, 30, 6, 40, 2,vecObs,0);
	_Obs3.simulateObstacles(37, 0, 48, 6, 40, 2,vecObs,0);


	//Create the vectors with initial poses and distances between vertex
	mcfg.loadData(X1,Y1,Z1,X2,Y2,Z2);
	printf("\nValues of Initial Trajectory Loaded \n");
	printf("\nTotal Number of Vertex = %lu \n\n",mcfg.pose_vec_.size());

	
	//Create Objet for kdtree
	NearNeighbor _NN(vecObs);
	mcfg.getSlopXYZAxes(mXYZ_);

	//////////////// Create Vertex. Points of the trajectory ////////////////
	//To save points cable before and after optimization
	file_in_pos.open (path+"initial_trajectory.txt");
	for (size_t i = 0; i < mcfg.pose_vec_.size(); ++i)
	{	
		vertex1 = new g2o::VertexXYZ();
		Eigen::Vector3d xyz = mcfg.pose_vec_[i].position;
		vertex1->setId(mcfg.pose_vec_[i].id);
		vertex1->setEstimate(xyz);
		//First and last vertex are fixed. 	
		if (i == 0 || i == mcfg.pose_vec_.size()-1)
			vertex1->setFixed(true);
		optimizer.addVertex(vertex1);	
		file_in_pos << setprecision(6) << xyz.x() << ";" << xyz.y() << ";" << xyz.z()<< endl;
	}
	file_in_pos.close();	


	// I. Add edges. These are the distances between the poses.
	for (size_t i = 0; i < mcfg.dist_vec_.size(); ++i)
	{
		g2o::G2ODistanceVertexEdge* edge = new g2o::G2ODistanceVertexEdge();
		edge->vertices()[0] = optimizer.vertices()[mcfg.dist_vec_[i].id_from];
		edge->vertices()[1] = optimizer.vertices()[mcfg.dist_vec_[i].id_to];
		edge->setMeasurement(mcfg.d3D_/mcfg.n_points);
		edge->setInformation(G2ODistanceVertexEdge::InformationType::Identity()*w_alpha); //UPDATE
		optimizer.addEdge(edge);
	}
	
	// II. Add edges. These are the distances from vertex to obstacles poses.
	for (size_t i = 0; i < mcfg.pose_vec_.size(); ++i)
	{
		g2o::G2OObstaclesEdge* edge = new g2o::G2OObstaclesEdge();
		edge->vertices()[0] = optimizer.vertices()[i];
		edge->readKDTree(_NN.kdtree,_NN.obs_points);
		edge->setMeasurement(bound);
		edge->setInformation(G2OObstaclesEdge::InformationType::Identity()*w_beta);//UPDATE
		optimizer.addEdge(edge);
	}

	// III. Add edges. These are the X distances between Vertices.
	for (size_t i = 0; i < mcfg.dist_vec_.size(); ++i)
	{
		g2o::G2ODistanceXYZEdge* edge = new g2o::G2ODistanceXYZEdge();
		edge->vertices()[0] = optimizer.vertices()[mcfg.dist_vec_[i].id_from];
		edge->vertices()[1] = optimizer.vertices()[mcfg.dist_vec_[i].id_to];
		edge->setMeasurement(mXYZ_);
		edge->setInformation(G2ODistanceXYZEdge::InformationType::Identity()*w_gamma); //UPDATE
		optimizer.addEdge(edge);
	}

	// // IV. Add edges. This is kinematic of trajectory.
	for (size_t i = 0; i < mcfg.pose_vec_.size()-3; ++i)
	{
		G2OKinematicsEdge* edge = new G2OKinematicsEdge;
		edge->vertices()[0] = optimizer.vertices()[i];
		edge->vertices()[1] = optimizer.vertices()[i+1];
		edge->vertices()[2] = optimizer.vertices()[i+2];
		// edge->vertices()[3] = optimizer.vertices()[i+3];
		edge->setMeasurement(angle);
		edge->setInformation(G2OKinematicsEdge::InformationType::Identity()*w_delta);
		optimizer.addEdge(edge);
	}

	// V. Add edges. This is Equi-distance among vertices.
	for (size_t i = 0; i < mcfg.pose_vec_.size()-3; ++i)
	{
		G2OEquiDistanceEdge* edge = new G2OEquiDistanceEdge;
		edge->vertices()[0] = optimizer.vertices()[i];
		edge->vertices()[1] = optimizer.vertices()[i+1];
		edge->vertices()[2] = optimizer.vertices()[i+2];
		edge->setInformation(G2OEquiDistanceEdge::InformationType::Identity()*w_eta);
		optimizer.addEdge(edge);
	}
	//////////////// Create Vertex. Time between consecutive configuration ////////////////
	// To save temporal state before optimization
	file_in_time.open ((path+"initial_time.txt"));
	file_in_velocity.open ((path+"initial_velocity.txt"));
	file_in_difftime.open ((path+"initial_difftime.txt"));
	double _dT = 0.0; 
	double _sum_dT = 0.0;
	double _sum_dist = 0.0;
	for (size_t i = 0; i < mcfg.dist_vec_.size(); ++i)
	{	
		_dT = mcfg.time_vec_[i].time;
		_sum_dT = _dT + _sum_dT;
		_sum_dist = mcfg.dist_vec_[i].dist + _sum_dist;
		vertex2 = new g2o::VertexTimeDiff();
		if (i == 0)	//First vertex is fixed. 
			vertex2 = new g2o::VertexTimeDiff(_dT,true);
			// vertex2->setFixed(true);
		else
			vertex2 = new g2o::VertexTimeDiff(_dT,false);
		
			// vertex2->setFixed(false);
		vertex2->setId(i+mcfg.dist_vec_.size()+1);
		// vertex2->setEstimate(_dT);
		optimizer.addVertex(vertex2);	
		file_in_time << setprecision(6) << _sum_dist << ";" << _sum_dT << endl;
		file_in_velocity  << setprecision(6) << _sum_dist << ";" << velocity << endl;
		file_in_difftime << setprecision(6) << _sum_dist << ";" << _dT << endl;
	}
	file_in_time.close();	
	file_in_velocity.close();	
	file_in_difftime.close();	

	// VI. Add edges. These are time between Vertices.
	for (unsigned i=mcfg.dist_vec_.size()+1; i < mcfg.dist_vec_.size()*2.0; ++i)
	{
		G2OTimeEdge* edge = new G2OTimeEdge;
		edge->vertices()[0] = optimizer.vertices()[i];
		edge->setMeasurement(mcfg.time_vec_[i-mcfg.dist_vec_.size()-1].time);
		edge->setInformation(G2OTimeEdge::InformationType::Identity()*w_epsilon);
		optimizer.addEdge(edge);
	}

	// VII. Add edges. These are velocities between Vertices.
	for (size_t i = 0; i < mcfg.dist_vec_.size(); ++i)
	{
		G2OVelocityEdge* edge = new G2OVelocityEdge;
		edge->vertices()[0] = optimizer.vertices()[mcfg.dist_vec_[i].id_from];
		edge->vertices()[1] = optimizer.vertices()[mcfg.dist_vec_[i].id_to];
		edge->vertices()[2] = optimizer.vertices()[i+mcfg.dist_vec_.size()+1];
		edge->setMeasurement(velocity);
		edge->setInformation(G2OVelocityEdge::InformationType::Identity()*w_zeta);
		optimizer.addEdge(edge);
	}

	// VIII. Add edges. This is the trajectory distances, between first and last vertex.
	// {
	// 	g2o::G2ODistanceTrajectoryEdge* edge = new g2o::G2ODistanceTrajectoryEdge();
	// 	edge->resize(poses.size());
	// 	for (size_t i = 0; i < poses.size(); ++i)
	// 	{
	// 		edge->vertices()[i] = optimizer.vertices()[i];
	// 	}
	// 	edge->setMeasurement(distance3D);
	// 	edge->setSizeTrajectory(poses.size());
	// 	edge->setInformation(G2ODistanceTrajectoryEdge::InformationType::Identity()*w_delta); //UPDATE
	// 	optimizer.addEdge(edge);
	// }

	// IX. Add edges. These are the distances from vertex to initial trayectory.
	// for (size_t i = 0; i < poses.size(); ++i)
	// {
	// 	g2o::g2o_trayectory_edge* edge = new g2o::g2o_trayectory_edge();
	// 	edge->vertices()[0] = optimizer.vertices()[i];
	// 	edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(poses[i].id)));
	// 	edge->setMeasurement(pos_);
	// 	edge->setInformation(g2o_trayectory_edge::InformationType::Identity()*w_gamma);//UPDATE
	// 	optimizer.addEdge(edge);
	// }

	std::cout << "Optimization  started !!" << std::endl;
	optimizer.save("antithing_before.g2o");
	optimizer.initializeOptimization();
	optimizer.setVerbose(true);
	optimizer.optimize(n_inter_opt);
	optimizer.save("antithing_after.g2o");


	//Save Optimized Trajectory in File.txt 
	file_out_pos.open (path+"optimized_trajectory.txt");
	for (unsigned i = 0; i < mcfg.pose_vec_.size(); i++)
	{
		structPose vOp;
		g2o::VertexXYZ *pose = dynamic_cast<g2o::VertexXYZ*>(optimizer.vertex(i));
		vOp.position = pose->estimate();	
		vOp.id = i;
		file_out_pos << setprecision(6) << vOp.position.x() << ";" << vOp.position.y() << ";" << vOp.position.z()<< endl;
	}
	file_out_pos.close();

	//Save Optimized Time in File.txt 
	file_out_time.open (path+"optimized_time.txt");
	file_out_velocity.open (path+"optimized_velocity.txt");
	file_out_difftime.open (path+"optimized_difftime.txt");
	double sum_dispos = 0.0;
	double sum_diffTime = 0.0;
	double velocity = 0.0;
 	for (size_t i = 0; i < mcfg.dist_vec_.size(); i++)
	{
		g2o::VertexXYZ *pose1Op = dynamic_cast<g2o::VertexXYZ*>(optimizer.vertex(i));
		g2o::VertexXYZ *pose2Op = dynamic_cast<g2o::VertexXYZ*>(optimizer.vertex(i+1));
		g2o::VertexTimeDiff *difftime = dynamic_cast<g2o::VertexTimeDiff*>(optimizer.vertex(i+mcfg.dist_vec_.size()+1));
		double distance = (pose2Op->estimate()-pose1Op->estimate()).norm();	
		sum_dispos = distance + sum_dispos;
		sum_diffTime = difftime->estimate()+ sum_diffTime;
		velocity = distance / difftime->estimate();
		file_out_time << setprecision(6) << sum_dispos << ";" << sum_diffTime << endl;
		file_out_velocity << setprecision(6) << sum_dispos << ";" << velocity << endl;
		file_out_difftime << setprecision(6) << sum_dispos << ";" << difftime->estimate() << endl;
		// printf("p1=[%f %f %f]  p2=[%f %f %f] d=%f Sum_d=%f\n",pose1Op->estimate().x(),pose1Op->estimate().y(),pose1Op->estimate().z(),pose2Op->estimate().x(),pose2Op->estimate().y(),pose2Op->estimate().z(),distance,sum_dispos);
	}
	file_out_time.close();
	file_out_velocity.close();
	file_out_difftime.close();

	std::cout << "Optimization Proccess  Completed !!!" << std::endl;
	optimizer.clear();
}

#endif
