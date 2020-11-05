/*
Optimizer Local Planner based in g2o for Marsupial Robotics COnfiguration 
Simon Martinez Rozas, 2020 
Service Robotics Lab, University Pablo de Olavide , Seville, Spain 
 */

#include "marsupial_g2o/optimizer_local_planner.h"

OptimizerLocalPlanner::OptimizerLocalPlanner(tf2_ros::Buffer *tfBuffer_)
{
	nh.reset(new ros::NodeHandle("~"));
	tf_list_ptr.reset(new tf::TransformListener(ros::Duration(5)));
    tfBuffer = tfBuffer_;
	
	resetFlags();

	nh.reset(new ros::NodeHandle("~"));
    tf_list.reset(new tf::TransformListener);

	nh->param<double>("w_alpha", w_alpha,0.1);
	nh->param<double>("w_beta",	w_beta,0.1);
	nh->param<double>("w_iota",	w_iota,0.1);
  	nh->param<double>("w_gamma", w_gamma,0.1);
  	nh->param<double>("w_delta", w_delta,0.1);
  	nh->param<double>("w_epsilon", w_epsilon,0.1);
  	nh->param<double>("w_zeta", w_zeta,0.1);
  	nh->param<double>("w_eta", w_eta,0.1);
  	nh->param<double>("w_theta", w_theta,0.1);
  	nh->param<double>("w_kappa", w_kappa,0.1);
  	nh->param<double>("w_lambda", w_lambda,0.1);

  	nh->param<double>("initial_multiplicative_factor_length_catenary", initial_multiplicative_factor_length_catenary,1.0); //Can't be lower than 1.0 or "catenary < distance between UGV and vertex" 	

	nh->param<int>("n_iter_opt", n_iter_opt,200);
  	nh->param<double>("distance_obstacle", distance_obstacle,2.0);
  	nh->param<double>("velocity", velocity,1.0);
  	nh->param<double>("acceleration", acceleration,0.0);
  	nh->param<double>("angle_min_traj", angle_min_traj, M_PI / 15.0);
  	nh->param<double>("distance_catenary_obstacle", distance_catenary_obstacle, 0.1);
  	nh->param<double>("dynamic_catenary", dynamic_catenary, 0.5);
  	nh->param<double>("min_distance_add_new_point", min_distance_add_new_point, 0.5);
  	nh->param<double>("bound_bisection_a", bound_bisection_a,100.0);
  	nh->param<double>("bound_bisection_b", bound_bisection_b,100.0);

  	nh->param<double>("z_constraint", z_constraint,0.0);

  	nh->param<int>("n_time_optimize", n_time_optimize, 1);
  	nh->param<double>("td_", td_, 0.5);
	
  	nh->param<bool>("verbose_optimizer",verbose_optimizer, true);

	
  	nh->param<bool>("write_data_for_analysis",write_data_for_analysis, false);
	nh->param("path", path, (std::string) "/home/simon/");
	nh->param("name_output_file", name_output_file, (std::string) "optimization_test");
	nh->param<int>("stage_number", stage_number, 1);
	nh->param<int>("num_pos_initial", num_pos_initial, 1);
	nh->param<int>("num_goal", num_goal, 0);
		
  	// nh->param("uav_base_frame", uav_base_frame, (std::string)"uav_base_link");
  	// nh->param("ugv_base_frame", ugv_base_frame, (std::string)"ugv_base_link");

	nh->param("debug", debug, (bool)0);
    nh->param("show_config", showConfig, (bool)0);

    ROS_INFO_COND(showConfig, PRINTF_GREEN "Optimizer Local Planner 3D Node Configuration:\n");
    // ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Trajectory Position Increments = [%.2f], Tolerance: [%.2f]", traj_dxy_max, traj_pos_tol);
    // ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Robot base frame: %s, World frame: %s", uav_base_frame.c_str(), world_frame.c_str());
    // ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Workspace:\t X:[%.2f, %.2f]\t Y:[%.2f, %.2f]\t Z: [%.2f, %.2f]", ws_x_max, ws_x_min, ws_y_max, ws_y_min, ws_z_max, ws_z_min);
    // ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Map Resolution: %.2f\t Map H inflaction: %.2f\t Map V Inflaction: %.2f", map_resolution, map_h_inflaction, map_v_inflaction);
    // ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Z weight cost: %.2f\t Z not inflate: %.2f", z_weight_cost, z_not_inflate);
	

	initializeSubscribers();
	initializePublishers();
	setupOptimizer();
	cleanVectors();
	ROS_INFO("alpha=[%f] beta=[%f] gamma=[%f] delta=[%f] epsilon=[%f] zeta=[%f] eta=[%f] theta=[%f]",w_alpha,w_beta,w_gamma,w_delta,w_epsilon,w_zeta,w_eta,w_theta);
	ROS_INFO("Optimizer_Local_Planner: Parameters loaded, ready for Optimization Process. Waiting for Action!!");
    // clearMarkers(0);
	traj_marker_pub_.publish(points_marker);
	traj_marker_pub_.publish(lines_marker);
	configServices();
}

void OptimizerLocalPlanner::initializeSubscribers()
{
	bool useOctomap;
    nh->param("use_octomap", useOctomap, (bool)false);
    if (useOctomap)
    {
        local_map_sub = nh->subscribe<octomap_msgs::Octomap>("/octomap_binary_local", 1, &OptimizerLocalPlanner::collisionMapCallBack, this);
    }
    else
    {
        // local_map_sub = nh->subscribe<PointCloud>("/points", 1, &OptimizerLocalPlanner::pointsSub, this);
    }
    octomap_sub_ = nh->subscribe( "/octomap_point_cloud_centers", 1,  &OptimizerLocalPlanner::readOctomapCallback, this);
    ROS_INFO("Optimizer_Local_Planner: Subscribers Initialized");
}

void OptimizerLocalPlanner::initializePublishers()
{
    traj_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("opt_trajectory_marker", 2);
	catenary_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("catenary_marker", 100);

	// visMarkersPublisher = nh->advertise<visualization_msgs::Marker>("markers", 1, true);
    // trajPub = nh->advertise<trajectory_msgs::MultiDOFJointTrajectory>("local_path", 1);

    ROS_INFO("Optimizer_Local_Planner: Publishers Initialized");
}

void OptimizerLocalPlanner::resetFlags()
{
    mapReceived = false;
	continue_optimizing = false;
}

void OptimizerLocalPlanner::cleanVectors()
{
	vec_time_init.clear();
	vec_dist_init.clear();
	vec_pose_init.clear();
	vec_len_cat_init.clear();
}

void OptimizerLocalPlanner::configServices()
{
    execute_path_srv_ptr.reset(new ExecutePathServer(*nh, "/Execute_Plan", false));
    execute_path_srv_ptr->registerGoalCallback(boost::bind(&OptimizerLocalPlanner::executeOptimizerPathGoalCB, this));
    execute_path_srv_ptr->registerPreemptCallback(boost::bind(&OptimizerLocalPlanner::executeOptimizerPathPreemptCB, this));
    execute_path_srv_ptr->start();

    // navigation3DClient.reset(new Navigate3DClient("/Navigation3D", true));
    // navigation3DClient->waitForServer();
	// ROS_ERROR("ENTER configServices Optimizer_Local_Planner 3");

}

void OptimizerLocalPlanner::setupOptimizer()
{
	//! Create the linear solver (Ax = b)
	// auto linearSolver = g2o::make_unique<LinearSolverCSparse<BlockSolverX::PoseMatrixType>>();
    auto linearSolver = g2o::make_unique<LinearSolverDense<BlockSolverX::PoseMatrixType>>(); // Linear equation solver

	auto blockSolver = g2o::make_unique<BlockSolverX>(move(linearSolver));
	
	OptimizationAlgorithmLevenberg* optimizationAlgorithm = new OptimizationAlgorithmLevenberg(move(blockSolver));
	// OptimizationAlgorithmGaussNewton* optimizationAlgorithm = new g2o::OptimizationAlgorithmGaussNewton(move(blockSolver));
    // OptimizationAlgorithmDogleg* optimizationAlgorithm = new OptimizationAlgorithmDogleg(move(blockSolver));

	// optimizationAlgorithm->setMaxTrialsAfterFailure(3);
	optimizationAlgorithm->setUserLambdaInit(0.00001);

	optimizer.clear();
	optimizer.setAlgorithm(optimizationAlgorithm);
    optimizer.setVerbose(verbose_optimizer);

	// getSlopXYZAxes(v_mXYZ_);
}

void OptimizerLocalPlanner::dynRecCb(marsupial_g2o::OptimizationParamsConfig &config, uint32_t level)
{
    // this->cost_weight = config.cost_weight;
    // this->lof_distance = config.lof_distance;
    // this->occ_threshold = config.occ_threshold;
    // this->goal_weight = config.goal_weight;
}

void OptimizerLocalPlanner::readOctomapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	  nn_.setInput(*msg);
}

void OptimizerLocalPlanner::collisionMapCallBack(const octomap_msgs::OctomapConstPtr &msg)
{
    mapReceived = true;
    //map = msg;
    // theta3D.updateMap(msg);
    // ROS_INFO_COND(debug, PRINTF_MAGENTA "Collision Map Received");
    // theta3D.publishOccupationMarkersMap();
}

// void OptimizerLocalPlanner::pointsSub(const PointCloud::ConstPtr &points)
// {
//     // ROS_INFO_COND(debug, PRINTF_MAGENTA "Collision Map Received");
//     mapReceived = true;
//     PointCloud out;
//     pcl_ros::transformPointCloud(uav_base_frame, *points, out, *tf_list_ptr);
//     // theta3D.updateMap(out);
//     // theta3D.publishOccupationMarkersMap();
// }

void OptimizerLocalPlanner::executeOptimizerPathPreemptCB()
{
    ROS_INFO_COND(debug, "Goal Preempted");
    execute_path_srv_ptr->setPreempted(); // set the action state to preempted

    // navigation3DClient->cancelAllGoals();

    resetFlags();
    clearMarkers(0);
}

void OptimizerLocalPlanner::executeOptimizerPathGoalCB()
{
    ROS_INFO_COND(debug, PRINTF_GREEN "Optimizer Local Planner Goal received in action server mode");
    //upo_actions::ExecutePathGoalConstPtr path_shared_ptr;
    auto path_shared_ptr = execute_path_srv_ptr->acceptNewGoal();
    globalTrajectory = path_shared_ptr->path;
	vector<float> length_catenary_initial;
	length_catenary_initial = path_shared_ptr->length_catenary;
	printf("length_catenary_initial.size=[%lu]\n",length_catenary_initial.size());

	for (size_t i = 0; i < globalTrajectory.points.size(); i++){
		printf("point=[%lu/%lu] GlobalTrayectoryPointTranslation=[%f %f %f]\n",i+1, globalTrajectory.points.size(),
		globalTrajectory.points.at(i).transforms[0].translation.x, globalTrajectory.points.at(i).transforms[0].translation.y, globalTrajectory.points.at(i).transforms[0].translation.z);
	}
		
	getPointsFromGlobalPath(globalTrajectory,new_path_from_global);
    auto size = new_path_from_global.size();
    ROS_INFO_COND(debug, PRINTF_GREEN "Number of Vertices to optimize = [%lu] = size",size);

	// setInitialLengthCatenaryAndPosUGV(v_initial_length_catenary, size);
	// ros::Duration(4.0).sleep();
	preComputeLengthCatenary(v_pre_initial_length_catenary, size);
	printf("ugv_pos_reel_catenary=[%f %f %f]\n",ugv_pos_catenary.x,ugv_pos_catenary.y,ugv_pos_catenary.z);
	checkObstaclesBetweenCatenaries(v_pre_initial_length_catenary,v_initial_length_catenary,size);
	ros::Duration(6.0).sleep();
	clearMarkers(_catenary_marker,50);
	ros::Duration(1.0).sleep();

	count_edges = 0;

	for (int j_ = 0; j_ < n_time_optimize; j_++)
	{
		// resetFlags();
    	clearMarkers(size);
		calculateDistanceVertices(new_path_from_global,vec_dist_init);

		if (!continue_optimizing){
			std::cout << std::endl <<  "==================================================="  << std::endl 
			<< "Preparing to execute Optimization: Creating Vertices and Edges!!" << std::endl;
		}

		/*
		* I Create Vertex: WayPoints for local trajectory to optimize 
		*/
		for (size_t i = 0; i < size; i++){	
			vertexPose = new g2o::VertexPointXYZ();
			vertexPose->setEstimate(new_path_from_global[i]);
			if (i == 0 || i == size-1)		//First and last vertices are fixed.	
				vertexPose->setFixed(true);
			else
				vertexPose->setFixed(false);
			vertexPose->setId(i);
			optimizer.addVertex(vertexPose);
		}
		//! I. Add Edges: Distances between two Vertices.
		// for (size_t i = 0; i < size-1; i++)
		// {
		// 	edgeDistanceVertex = new g2o::G2ODistanceVertexEdge();
		// 	edgeDistanceVertex->vertices()[0] = optimizer.vertices()[i];
		// 	edgeDistanceVertex->vertices()[1] = optimizer.vertices()[i+1];
		// 	edgeDistanceVertex->setMeasurement(d3D_/n_points);
		// 	edgeDistanceVertex->setInformation(G2ODistanceVertexEdge::InformationType::Identity()*w_alpha); 
		// 	optimizer.addEdge(edgeDistanceVertex);
		//	count_edges++;
		// }
		// //! II. Add Edges: Distances in XYZ axes between two Vertices.
		// for (size_t i = 0; i < size-1; i++)
		// {
		// 	edgeDistanceXYZ = new g2o::G2ODistanceXYZEdge();
		// 	edgeDistanceXYZ->vertices()[0] = optimizer.vertices()[i];
		// 	edgeDistanceXYZ->vertices()[1] = optimizer.vertices()[i+1];
		// 	edgeDistanceXYZ->setMeasurement(v_mXYZ_);
		// 	edgeDistanceXYZ->setInformation(G2ODistanceXYZEdge::InformationType::Identity()*w_gamma); 
		// 	optimizer.addEdge(edgeDistanceXYZ);
		//	count_edges++;
		// }
		// ! III. Add Edges: Equi-distance among vertices.
		for (size_t i = 0; i < size-3; i++){
			edgeEquiDistance = new g2o::G2OEquiDistanceEdge;
			edgeEquiDistance->vertices()[0] = optimizer.vertices()[i];
			edgeEquiDistance->vertices()[1] = optimizer.vertices()[i+1];
			edgeEquiDistance->vertices()[2] = optimizer.vertices()[i+2];
			edgeEquiDistance->vertices()[3] = optimizer.vertices()[i+3];
			edgeEquiDistance->setInformation(G2OEquiDistanceEdge::InformationType::Identity()*w_epsilon);
			optimizer.addEdge(edgeEquiDistance);
			count_edges++;
		}
		//! IV. Add Edges: Nearest Obstacles distances from Vertex.
		for (size_t i = 0; i < size; i++){
			edgeObstaclesNear = new g2o::G2OObstaclesEdge();
			edgeObstaclesNear->vertices()[0] = optimizer.vertices()[i];
			edgeObstaclesNear->readKDTree(nn_.kdtree,nn_.obs_points);
			printf("distance_obstacle=[%]\n",distance_obstacle);
			edgeObstaclesNear->setMeasurement(distance_obstacle);
			edgeObstaclesNear->setInformation(G2OObstaclesEdge::InformationType::Identity()*w_beta);
			optimizer.addEdge(edgeObstaclesNear);
			count_edges++;
		}
		//! V. Add Edges: Obstacles between two Vertices.
		for (size_t i = 0; i < size-1; i++){
			edgeThroughObstacles = new g2o::G2OThroughObstaclesEdge();
			edgeThroughObstacles->vertices()[0] = optimizer.vertices()[i];
			edgeThroughObstacles->vertices()[1] = optimizer.vertices()[i+1];
			edgeThroughObstacles->setRobustKernel(new g2o::RobustKernelCauchy());
            edgeThroughObstacles->robustKernel()->setDelta(1.0);
			edgeThroughObstacles->readKDTree(nn_.kdtree,nn_.obs_points);
			edgeThroughObstacles->setInformation(G2OThroughObstaclesEdge::InformationType::Identity()*w_iota); 
			optimizer.addEdge(edgeThroughObstacles);
			count_edges++;
		}
		// VI. Add Edges: Kinematic of trajectory.
		for (size_t i = 0; i < size-2; i++){
			edgeKinetics = new g2o::G2OKinematicsEdge;
			edgeKinetics->vertices()[0] = optimizer.vertices()[i];
			edgeKinetics->vertices()[1] = optimizer.vertices()[i+1];
			edgeKinetics->vertices()[2] = optimizer.vertices()[i+2];
			edgeKinetics->setMeasurement(angle_min_traj);
			edgeKinetics->setInformation(G2OKinematicsEdge::InformationType::Identity()*w_delta);
			optimizer.addEdge(edgeKinetics);
			count_edges++;
		}

		/*
		* II Create Vertex: Time between consecutive waypoints in trajectory 
		*/
		getTemporalState(vec_time_init,vec_dist_init,velocity);
		for (size_t i = 0; i < size-1; i++){	
			vertexTime = new g2o::VertexTimeDiff();
			vertexTime->setEstimate(vec_time_init[i].time);
			vertexTime->setFixed(false);
			vertexTime->setId(i+size);
			optimizer.addVertex(vertexTime);
		}
		
		if (!continue_optimizing)
			writeTemporalDataBeforeOptimization();

		//! VII. Add edges. These are time between Vertices.
		for (size_t i=size; i < size*2.0-1.0; i++){
			G2OTimeEdge* edgeTime = new G2OTimeEdge;
			edgeTime->vertices()[0] = optimizer.vertices()[i];
			edgeTime->setMeasurement(vec_time_init[i-size].time);
			edgeTime->setInformation(G2OTimeEdge::InformationType::Identity()*w_zeta);
			optimizer.addEdge(edgeTime);
			count_edges++;
		}
		//! VIII. Add edges. These are velocities between Vertices.
		for (size_t i = size; i < size*2.0-1.0; i++){
			G2OVelocityEdge* edgeVelocity = new G2OVelocityEdge;
			edgeVelocity->vertices()[0] = optimizer.vertices()[i-size];
			edgeVelocity->vertices()[1] = optimizer.vertices()[i-size+1];
			edgeVelocity->vertices()[2] = optimizer.vertices()[i];
			edgeVelocity->setMeasurement(velocity);
			edgeVelocity->setInformation(G2OVelocityEdge::InformationType::Identity()*w_eta);
			optimizer.addEdge(edgeVelocity);
			count_edges++;
		}
		// ! IX. Add edges. This is aceleration between Vertices.
		for (size_t i = size; i < size*2.0-2.0; i++){
			G2OAccelerationEdge* edgeAcceleration = new G2OAccelerationEdge;
			edgeAcceleration->vertices()[0] = optimizer.vertices()[i-size];
			edgeAcceleration->vertices()[1] = optimizer.vertices()[i-size+1];
			edgeAcceleration->vertices()[2] = optimizer.vertices()[i-size+2];
			edgeAcceleration->vertices()[3] = optimizer.vertices()[i];
			edgeAcceleration->vertices()[4] = optimizer.vertices()[i+1];
			edgeAcceleration->setMeasurement(acceleration);
			edgeAcceleration->setInformation(G2OAccelerationEdge::InformationType::Identity()*w_theta);
			optimizer.addEdge(edgeAcceleration);
			count_edges++;
		}

		/*
		* III Create Vertex: Multiplicative factor for Catenary Length
		*/
		for (size_t i = 0; i < size; i++){	
			vertexCatenaryLength = new g2o:: VertexCatenaryLength();
			vertexCatenaryLength->setEstimate(v_initial_length_catenary[i]);
			vertexCatenaryLength->setFixed(false);
			vertexCatenaryLength->setId(i+size+size-1);
			optimizer.addVertex(vertexCatenaryLength);
		}

		n_edges_before_catenary = count_edges;	
		//! X. Add Edges: Catenary chain.
		for (size_t i = 0; i < size; i++){
			G2OCatenaryEdge* edgeCatenary = new g2o::G2OCatenaryEdge(nh);
			edgeCatenary->vertices()[0] = optimizer.vertices()[i];
			edgeCatenary->vertices()[1] = optimizer.vertices()[i+size-1.0+size];
			edgeCatenary->readKDTree(nn_.kdtree,nn_.obs_points);
			edgeCatenary->setRadius(distance_catenary_obstacle);
			edgeCatenary->numberVerticesNotFixed(size);
			edgeCatenary->setInitialPosCatUGV(ugv_pos_catenary);
			edgeCatenary->setMeasurement(v_initial_length_catenary);
			edgeCatenary->setBoundForBisection(bound_bisection_a,bound_bisection_b);
			edgeCatenary->setBoundZNegative(z_constraint);
			edgeCatenary->setRobustKernel(new g2o::RobustKernelCauchy());
            edgeCatenary->robustKernel()->setDelta(1.0);
			edgeCatenary->setInformation(G2OCatenaryEdge::InformationType::Identity()*w_kappa);
			optimizer.addEdge(edgeCatenary);
			count_edges++;
		}
		//! XI. Add Edges: Dynamic folding and unfolding tether.
		for (size_t i = 0; i < size-1; i++){
			G2ODynamicCatenaryEdge* edgeDynamicCatenary = new g2o::G2ODynamicCatenaryEdge();
			edgeDynamicCatenary->vertices()[0] = optimizer.vertices()[i+size-1.0+size];
			edgeDynamicCatenary->vertices()[1] = optimizer.vertices()[i+size-1.0+size+1];
			edgeDynamicCatenary->vertices()[2] = optimizer.vertices()[i+size];
			edgeDynamicCatenary->vertices()[3] = optimizer.vertices()[i+size+1];
			edgeDynamicCatenary->setMeasurement(dynamic_catenary);
			edgeDynamicCatenary->setInformation(G2ODynamicCatenaryEdge::InformationType::Identity()*w_lambda); 
			optimizer.addEdge(edgeDynamicCatenary);
			count_edges++;
		}

		if (!continue_optimizing)
			std::cout <<  "==================================================="  << std::endl << "Optimization  started !!" << std::endl;
		
		start_time_opt_ = ros::Time::now();
		// optimizer.save("antithing_before.g2o");
		optimizer.initializeOptimization();
		optimizer.setVerbose(true);
		optimizer.optimize(n_iter_opt);
		// optimizer.save("antithing_after.g2o");
		final_time_opt_ = ros::Time::now();

		if (j_ + 1 == n_time_optimize)
			std::cout << std::endl <<"Optimization Proccess Completed !!!" << std::endl << "===================================================" << std::endl << "Saving Temporal data in txt file ..." << std::endl;

	
		vec_pose_opt.clear();
		vec_len_cat_opt.clear();
		vec_dist_opt.clear();
		vec_time_opt.clear();
		vec_vel_opt.clear();
		vec_acc_opt.clear();
		new_path_from_global.clear(); //clear the path to set the optimizer solution as path
		v_initial_length_catenary.clear();

		for (size_t i = 0; i < size; i++){
			structPose vOp;
			structLengthCatenary vLC;
			vOp.vertex = dynamic_cast<g2o::VertexPointXYZ*>(optimizer.vertex(i));
			vOp.position = vOp.vertex->estimate();	
			vOp.id = vOp.vertex->id();
			vec_pose_opt.push_back(vOp);
			new_path_from_global.push_back(vOp.position);
			optimizer.removeVertex(optimizer.vertex(vOp.vertex->id()));
			vLC.vertex = dynamic_cast<g2o::VertexCatenaryLength*>(optimizer.vertex(i+size+size-1));
			vLC.length = vLC.vertex->estimate();
			vLC.id = vLC.vertex->id();
			vec_len_cat_opt.push_back(vLC);
			v_initial_length_catenary.push_back(vLC.length);

			optimizer.removeVertex(optimizer.vertex(vLC.vertex->id()));
		}

		getMarkerPoints(points_marker,vec_pose_opt,"points_lines",j_);
		getMarkerLines(lines_marker,vec_pose_opt,"points_lines",j_);
		traj_marker_pub_.publish(points_marker);
		traj_marker_pub_.publish(lines_marker);

		writeTemporalDataAfterOptimization(size);
		for (size_t i = 0; i < size-1; i++){
			optimizer.removeVertex(optimizer.vertex(i+size));
		}

		ros::Duration(td_).sleep();

		// std::string n_, n_test;
		// n_t = std::to_string(num_pos_initial);
		// n_test_ = "test_"+n_+"/";
		output_file = path+name_output_file+"_stage_"+std::to_string(stage_number)+"_InitPos_"+std::to_string(num_pos_initial)+"_goal_"+std::to_string(num_goal)+".txt";
		ofs.open(output_file.c_str(), std::ofstream::app);
		if (write_data_for_analysis)
			getDataForOptimizerAnalysis();

		cleanVectors();		//Clear vector after optimization and previus the next iteration


		if (j_ + 1 == n_time_optimize){
			std::cout << "...Temporal data saved in txt file." << std::endl << std::endl;
			ROS_INFO("Optimizer Local Planner: Goal position successfully achieved");
			ros::Duration(10.0).sleep();
			action_result.arrived = true;
			execute_path_srv_ptr->setSucceeded(action_result);

			std::cout <<"Optimization Local Planner Proccess ended !!!!!!!!!!!!!!!!!!!!!!!" << std::endl << "==================================================="<< std::endl << std::endl;
			resetFlags();
		}
		else
		{
			continue_optimizing = true;
			auto n_loop = j_ + 1;
			std::cout << std::endl <<"Optimization number " << n_loop << " finished, starting new proccess !!!!!!!!!!!!!!!!!!!!!!!" << std::endl << std::endl;
			optimizer.clear();
		}

		for (size_t k_ = 0; k_ < v_initial_length_catenary.size(); k_++){
			printf("Optimized length_catenary=[%f] for vertex=[%lu]\n",v_initial_length_catenary[k_],k_);
		}
		std::cout << "==================================================="<< std::endl << std::endl;
	}
	optimizer.clear();
}

void OptimizerLocalPlanner::getDataForOptimizerAnalysis()
{
	bisectionCatenary bsCat;
	std::vector<geometry_msgs::Point> v_points_catenary_opt_,v_points_catenary_init_;

	// Writing general data for initial analysis
	double init_compute_time_, init_traj_distance_, init_traj_time_, init_traj_vel_, init_traj_vel_max_, init_traj_vel_mean_, init_traj_acc_,init_traj_acc_max_,init_traj_acc_mean_;
	init_compute_time_ = init_traj_distance_ = init_traj_time_ = init_traj_vel_ = init_traj_vel_max_ = init_traj_vel_mean_ = init_traj_acc_ = init_traj_acc_max_ = init_traj_acc_mean_=0.0;

	//Length and Time Trajectory Initial
	for(size_t j = 0 ; j < vec_time_init.size() ; j++){
		init_traj_time_ = vec_time_init[j].time + init_traj_time_;
		init_traj_distance_ = vec_dist_init[j].dist + init_traj_distance_;
		init_traj_vel_ = (vec_dist_init[j].dist / vec_time_init[j].time)+init_traj_vel_; 
		if (init_traj_vel_max_ < (vec_dist_init[j].dist / vec_time_init[j].time))
			init_traj_vel_max_ = (vec_dist_init[j].dist / vec_time_init[j].time);
	}
	init_traj_vel_mean_ = init_traj_vel_ / (double)vec_time_init.size();

	//Acceleration Trajectory Initial
	for(size_t j = 0 ; j < vec_time_init.size()-1 ; j++){
		init_traj_acc_ = ( ((vec_dist_init[j].dist / vec_time_init[j].time ) - (vec_dist_init[j+1].dist/vec_time_init[j+1].time)) / (vec_time_init[j].time +vec_time_init[j+1].time) ) + init_traj_acc_; ;
		if (init_traj_acc_max_ < ( ((vec_dist_init[j].dist / vec_time_init[j].time )- (vec_dist_init[j+1].dist/vec_time_init[j+1].time)) / (vec_time_init[j].time +vec_time_init[j+1].time) ) )
			init_traj_acc_max_ = ( ((vec_dist_init[j].dist / vec_time_init[j].time )- (vec_dist_init[j+1].dist/vec_time_init[j+1].time)) / (vec_time_init[j].time +vec_time_init[j+1].time) );
	}
	init_traj_acc_mean_ = init_traj_acc_ / (double)(vec_time_init.size()-1);

	//Distance Obstacles Initial
	double distance_obs_init_ , distance_obs_init_min_, distance_obs_init_mean_;
	distance_obs_init_ = distance_obs_init_mean_ = 0.0;
	distance_obs_init_min_ = 1000.0;
	for(size_t j = 0 ; j < vec_pose_init.size(); j ++){
		Eigen::Vector3d p_init_ = vec_pose_init[j];
		Eigen::Vector3d nearest_obs_p_ = nn_.nearestObstacleVertex(nn_.kdtree, p_init_ , nn_.obs_points);
		distance_obs_init_ = (p_init_- nearest_obs_p_).norm() + distance_obs_init_;
		if(distance_obs_init_min_ > (p_init_- nearest_obs_p_).norm() )
			distance_obs_init_min_ = (p_init_- nearest_obs_p_).norm();
	}
	distance_obs_init_mean_ = distance_obs_init_ / (double)vec_pose_init.size();

	//Distance Catenary Obstacles Initial
	double distance_obs_cat_init_ , distance_obs_cat_init_min_, distance_obs_cat_init_mean_;
	distance_obs_cat_init_ = distance_obs_cat_init_mean_ = 0.0;
	distance_obs_cat_init_min_ = 1000.0;
	int count_cat_p_init_ = 0;
	for(size_t j = 0 ; j < vec_pose_init.size(); j ++){
		bsCat.setNumberPointsCatenary(vec_len_cat_init[j].length*10.0);
		bsCat.setFactorBisection(bound_bisection_a,bound_bisection_b);
		bsCat.configBisection(vec_len_cat_init[j].length,ugv_pos_catenary.x,ugv_pos_catenary.y,ugv_pos_catenary.z,vec_pose_init[j].x(),vec_pose_init[j].y(),vec_pose_init[j].z(),j,"data_init_cat");
		v_points_catenary_init_.clear();
		bsCat.getPointCatenary3D(v_points_catenary_init_);

		int n_p_cat_dis_ = ceil(1.5*ceil(vec_len_cat_init[j].length)); // parameter to ignore collsion points in the begining and in the end of catenary
		if (n_p_cat_dis_ < 5)
			n_p_cat_dis_ = 5;

		for (size_t k= 0 ; k < v_points_catenary_init_.size() ; k++){
			if(k > n_p_cat_dis_ && k < v_points_catenary_init_.size()-floor(n_p_cat_dis_/2.0)){
				count_cat_p_init_++;
				Eigen::Vector3d p_cat_init_; 
				p_cat_init_.x()= v_points_catenary_init_[k].x;
				p_cat_init_.y()= v_points_catenary_init_[k].y;
				p_cat_init_.z()= v_points_catenary_init_[k].z;
				Eigen::Vector3d nearest_obs_p = nn_.nearestObstacleVertex(nn_.kdtree, p_cat_init_ , nn_.obs_points);
				distance_obs_cat_init_ = (p_cat_init_- nearest_obs_p).norm() + distance_obs_cat_init_;
				if(distance_obs_cat_init_min_ > (p_cat_init_- nearest_obs_p).norm() )
					distance_obs_cat_init_min_ = (p_cat_init_- nearest_obs_p).norm();
			}
			// printf("OBstacle Initial: vertex=[%lu/%lu] v_points_catenary_init_.size=[%lu] count_cat_p_init_=[%i] reel_pos=[%f %f %f] pos_vertex=[%f %f %f]\n",
			// j,vec_pose_init.size(),v_points_catenary_init_.size(),count_cat_p_init_,ugv_pos_catenary.x,ugv_pos_catenary.y,ugv_pos_catenary.z,vec_pose_init[j].x(),vec_pose_init[j].y(),vec_pose_init[j].z());
		}
	}
	distance_obs_cat_init_mean_ = distance_obs_cat_init_/ (double)count_cat_p_init_++;

	
	// Writing general data for optimized analysis
	double opt_compute_time_, opt_traj_distance_, opt_traj_time_, opt_traj_vel_, opt_traj_vel_max_, opt_traj_vel_mean_, opt_traj_acc_, opt_traj_acc_max_, opt_traj_acc_mean_;
	opt_compute_time_ = opt_traj_distance_ = opt_traj_time_ = opt_traj_vel_ = opt_traj_vel_max_ = opt_traj_vel_mean_ = opt_traj_acc_ = opt_traj_acc_max_ = opt_traj_acc_mean_ = 0.0;

	//Compute Time Optimization
	opt_compute_time_ = (final_time_opt_ - start_time_opt_).toSec();

	//Length, Time, Velocity Trajectory Optimized
	for (size_t j = 0; j < vec_time_opt.size() ; j++){
		opt_traj_time_ = vec_time_opt[j] + opt_traj_time_;
		opt_traj_distance_ = vec_dist_opt[j] + opt_traj_distance_;
		opt_traj_vel_ = vec_vel_opt[j] + opt_traj_vel_;
		if (opt_traj_vel_max_ < vec_vel_opt[j])
			opt_traj_vel_max_ = vec_vel_opt[j];
	}
	opt_traj_vel_mean_ = opt_traj_vel_ / (double)vec_time_opt.size();

	//Acceleration Trajectory Optimized
	for (size_t j = 0; j < vec_acc_opt.size() ; j++){
		opt_traj_acc_ = vec_acc_opt[j] + opt_traj_acc_;
		if (fabs(opt_traj_acc_max_) < fabs(vec_acc_opt[j]))
			opt_traj_acc_max_ = vec_acc_opt[j];
	}
	opt_traj_acc_mean_ = opt_traj_acc_ / (double)vec_acc_opt.size();


	//Distance Point Obstacles
	double distance_obs_opt_ , distance_obs_opt_min_, distance_obs_opt_mean_;
	distance_obs_opt_ = distance_obs_opt_mean_ = 0.0;
	distance_obs_opt_min_ = 1000.0;
	for(size_t j = 0 ; j < vec_pose_opt.size(); j ++){
		Eigen::Vector3d p_opt_ = vec_pose_opt[j].position;
		Eigen::Vector3d nearest_obs_p_ = nn_.nearestObstacleVertex(nn_.kdtree, p_opt_ , nn_.obs_points);
		distance_obs_opt_ = (p_opt_- nearest_obs_p_).norm() + distance_obs_opt_;
		if(distance_obs_opt_min_ > (p_opt_- nearest_obs_p_).norm() )
			distance_obs_opt_min_ = (p_opt_- nearest_obs_p_).norm();
	}
	distance_obs_opt_mean_ = distance_obs_opt_ / (double)vec_pose_opt.size();

	//Distance Catenary Obstacles Optimized
	double distance_obs_cat_opt_ , distance_obs_cat_opt_min_, distance_obs_cat_opt_mean_;
	distance_obs_cat_opt_ = distance_obs_cat_opt_mean_ = 0.0;
	distance_obs_cat_opt_min_ = 1000.0;
	int count_cat_p_opt_ = 0;

	for(size_t j = 0 ; j < vec_pose_opt.size(); j ++){
		bsCat.setNumberPointsCatenary(vec_len_cat_opt[j].length*10.0);
		bsCat.setFactorBisection(bound_bisection_a,bound_bisection_b);
		bsCat.configBisection(vec_len_cat_opt[j].length,ugv_pos_catenary.x,ugv_pos_catenary.y,ugv_pos_catenary.z,vec_pose_opt[j].position.x(),vec_pose_opt[j].position.y(),vec_pose_opt[j].position.z(),j,"data_opt_cat");
		v_points_catenary_opt_.clear();
		bsCat.getPointCatenary3D(v_points_catenary_opt_);

		int n_p_cat_dis_ = ceil(1.5*ceil(vec_len_cat_opt[j].length)); // parameter to ignore collsion points in the begining and in the end of catenary
		if (n_p_cat_dis_ < 5)
			n_p_cat_dis_ = 5;

		for (size_t k= 0 ; k < v_points_catenary_opt_.size() ; k++){
			if(k > n_p_cat_dis_ && k < v_points_catenary_opt_.size()-floor(n_p_cat_dis_/2.0)){
				count_cat_p_opt_++;
				Eigen::Vector3d p_cat_opt_; 
				p_cat_opt_.x()= v_points_catenary_opt_[k].x;
				p_cat_opt_.y()= v_points_catenary_opt_[k].y;
				p_cat_opt_.z()= v_points_catenary_opt_[k].z;
				Eigen::Vector3d nearest_obs_p = nn_.nearestObstacleVertex(nn_.kdtree, p_cat_opt_ , nn_.obs_points);
				distance_obs_cat_opt_ = (p_cat_opt_- nearest_obs_p).norm() + distance_obs_cat_opt_;
				if(distance_obs_cat_opt_min_ > (p_cat_opt_- nearest_obs_p).norm() )
					distance_obs_cat_opt_min_ = (p_cat_opt_- nearest_obs_p).norm();
			}
		// printf("Obstacles Optimized: vertex=[%lu/%lu] vec_pose_opt.size=[%lu] v_points_catenary_opt_.size=[%lu] count_cat_p_opt = [%i]\n",j, vec_pose_opt.size(),vec_pose_opt.size(),v_points_catenary_opt_.size(),count_cat_p_opt_);
		}
	}
	distance_obs_cat_opt_mean_ = distance_obs_cat_opt_/ (double)count_cat_p_opt_++;


	if (ofs.is_open()) {
		std::cout << "Saving data in output file: " << output_file << std::endl;
		ofs << opt_compute_time_ << "," 
		    << init_traj_distance_ << "," 
		    << opt_traj_distance_ << ","
			<< init_traj_time_ << "," 
			<< opt_traj_time_ << "," 
			<< distance_obs_init_mean_ << ","
			<< distance_obs_init_min_ << ","
			<< distance_obs_opt_mean_ << "," 
			<< distance_obs_opt_min_ << "," 
			<< distance_obs_cat_init_mean_ << "," 
			<< distance_obs_cat_init_min_ << "," 
			<< distance_obs_cat_opt_mean_ << "," 
			<< distance_obs_cat_opt_min_ << ","
			<< init_traj_vel_mean_ << "," 
			<< init_traj_vel_max_ << ","
			<< opt_traj_vel_mean_ << ","
			<< opt_traj_vel_max_ << ","
			<< init_traj_acc_mean_ << ","
			<< init_traj_acc_max_ << "," 
			<< opt_traj_acc_mean_ << ","
			<< opt_traj_acc_max_ 
			<<std::endl;
	} 
	else {
		std::cout << "Couldn't be open the output data file" << std::endl;
	}
	ofs.close();
}

void OptimizerLocalPlanner::getMarkerPoints(visualization_msgs::MarkerArray &marker_, vector<structPose> _vector, std::string ns, int j_)
{
	for (size_t i = 0; i < _vector.size(); i++){
		marker_.markers[i].header.frame_id = "/map";
		marker_.markers[i].header.stamp = ros::Time::now();
		marker_.markers[i].ns = "points";
		marker_.markers[i].id = i+1;
		marker_.markers[i].action = visualization_msgs::Marker::ADD;
		if (i % 5 == 0)
			marker_.markers[i].type = visualization_msgs::Marker::CUBE;
		else
			marker_.markers[i].type = visualization_msgs::Marker::SPHERE;
		if (j_+1 == n_time_optimize)
			marker_.markers[i].lifetime = ros::Duration(400);
		else
			marker_.markers[i].lifetime = ros::Duration(2.0);
		marker_.markers[i].pose.position.x = _vector[i].position.x();
		marker_.markers[i].pose.position.y = _vector[i].position.y();
		marker_.markers[i].pose.position.z = _vector[i].position.z();
		marker_.markers[i].pose.orientation.x = 0.0;
		marker_.markers[i].pose.orientation.y = 0.0;
		marker_.markers[i].pose.orientation.z = 0.0;
		marker_.markers[i].pose.orientation.w = 1.0;
		marker_.markers[i].scale.x = 0.2;
		marker_.markers[i].scale.y = 0.2;
		marker_.markers[i].scale.z = 0.2;
		marker_.markers[i].color.a = 1.0;
		marker_.markers[i].color.r = 0.0;
		marker_.markers[i].color.g = 0.0;
		marker_.markers[i].color.b = 0.9;
	}	
}

void OptimizerLocalPlanner::getMarkerLines(visualization_msgs::MarkerArray &marker_, vector<structPose> _vector, std::string ns, int j_)
{
	for (size_t i = 0; i < _vector.size()-1; i++){
		geometry_msgs::Point _p1, _p2; 
		marker_.markers[i].header.frame_id = "/map";
		marker_.markers[i].header.stamp = ros::Time::now();
		marker_.markers[i].ns = "points";
		marker_.markers[i].id = i + _vector.size()+2;
		marker_.markers[i].action = visualization_msgs::Marker::ADD;
		marker_.markers[i].type = visualization_msgs::Marker::LINE_STRIP;
		if (j_+1 == n_time_optimize)
			marker_.markers[i].lifetime = ros::Duration(400);
		else
			marker_.markers[i].lifetime = ros::Duration(2.0);
		_p1.x = _vector[i].position.x();
		_p1.y = _vector[i].position.y();
		_p1.z = _vector[i].position.z();
		marker_.markers[i].points.push_back(_p1);
		_p2.x = _vector[i+1].position.x();
		_p2.y = _vector[i+1].position.y();
		_p2.z = _vector[i+1].position.z();
		marker_.markers[i].points.push_back(_p2);
		marker_.markers[i].pose.orientation.x = 0.0;
		marker_.markers[i].pose.orientation.y = 0.0;
		marker_.markers[i].pose.orientation.z = 0.0;
		marker_.markers[i].pose.orientation.w = 1.0;
		marker_.markers[i].scale.x = 0.1;
		// marker_.markers[i].scale.y = 0.3;
		// marker_.markers[i].scale.z = 0.1;
		marker_.markers[i].color.a = 1.0;
		marker_.markers[i].color.r = 0.0;
		marker_.markers[i].color.g = 0.0;
		marker_.markers[i].color.b = 0.9;
	}
}

void OptimizerLocalPlanner::clearMarkers(auto _s)
{
	lines_marker.markers.clear();
	points_marker.markers.clear();

	lines_marker.markers.resize(_s);
	points_marker.markers.resize(_s);

	for (auto i = 0 ; i < _s; i++){
		points_marker.markers[i].action = visualization_msgs::Marker::DELETEALL;
		lines_marker.markers[i].action = visualization_msgs::Marker::DELETEALL;
	}
	traj_marker_pub_.publish(points_marker);
	traj_marker_pub_.publish(lines_marker);
}

void OptimizerLocalPlanner::getPointsFromGlobalPath(trajectory_msgs::MultiDOFJointTrajectory _path,vector<Eigen::Vector3d> &_v_gp)
{
	float x, y, z;
	Eigen::Vector3d _p;
    double D = 0.0;
	int n = 0;
	_v_gp.clear();
	int _count = 1; 
	
    for (size_t i = 0; i < _path.points.size()-1; i++)
    {
        x = _path.points.at(i+1).transforms[0].translation.x - _path.points.at(i).transforms[0].translation.x;
		y = _path.points.at(i+1).transforms[0].translation.y - _path.points.at(i).transforms[0].translation.y;
		z = _path.points.at(i+1).transforms[0].translation.z - _path.points.at(i).transforms[0].translation.z;
        D = sqrt(x * x + y * y + z * z);
		if (D > min_distance_add_new_point){
			n = floor(D / min_distance_add_new_point);
			double xp = x/((double)n+1.0);
			double yp = y/((double)n+1.0);
			double zp = z/((double)n+1.0);
			_p.x() = _path.points.at(i).transforms[0].translation.x;
			_p.y() = _path.points.at(i).transforms[0].translation.y;
			_p.z() = _path.points.at(i).transforms[0].translation.z;
			_v_gp.push_back(_p);
			printf("point =[%i] getPointsFromGlobalPath = [%f %f %f]\n",_count,_p.x(),_p.y(),_p.z());
			_count++;
			for (int j=0; j< n ; j++){
				_p = Eigen::Vector3d(0.0,0.0,0.0);
				_p.x() = _path.points.at(i).transforms[0].translation.x + xp*(1.0+(double)j);
				_p.y() = _path.points.at(i).transforms[0].translation.y + yp*(1.0+(double)j);
				_p.z() = _path.points.at(i).transforms[0].translation.z + zp*(1.0+(double)j);
				_v_gp.push_back(_p);
				printf("point =[%i] getPointsFromGlobalPath = [%f %f %f]\n",_count,_p.x(),_p.y(),_p.z());
				_count++;
			}
		}
		else{
			_p.x() = _path.points.at(i).transforms[0].translation.x;
			_p.y() = _path.points.at(i).transforms[0].translation.y;
			_p.z() = _path.points.at(i).transforms[0].translation.z;
			_v_gp.push_back(_p);
			printf("point =[%i] getPointsFromGlobalPath = [%f %f %f]\n",_count,_p.x(),_p.y(),_p.z());
			_count++;
		}
    }
	_p.x() = _path.points.at(_path.points.size()-1).transforms[0].translation.x;
	_p.y() = _path.points.at(_path.points.size()-1).transforms[0].translation.y;
	_p.z() = _path.points.at(_path.points.size()-1).transforms[0].translation.z;
	_v_gp.push_back(_p);
	printf("point =[%i] getPointsFromGlobalPath = [%f %f %f]\n",_count,_p.x(),_p.y(),_p.z());
	_count++;

	for (size_t i = 0 ; i <  _v_gp.size() ; i++){
		vec_pose_init.push_back(_v_gp[i]);
	}
}


void OptimizerLocalPlanner::calculateDistanceVertices(vector<Eigen::Vector3d> _path,vector<structDistance> &_v_sD)
{
	structDistance _sD;
	float x, y, z;
	_v_sD.clear();
    double pL = 0;
    for (size_t i = 0; i < _path.size()-1; i++)
    {
        x = new_path_from_global[i+1].x() - new_path_from_global[i].x();
		y = new_path_from_global[i+1].y() - new_path_from_global[i].y();
		z = new_path_from_global[i+1].z() - new_path_from_global[i].z();
        pL= sqrt(x * x + y * y + z * z);
		_sD.id_from = i;
		_sD.id_to = i+ 1;
		_sD.dist = pL;
		_v_sD.push_back(_sD);
    }
}

void OptimizerLocalPlanner::getTemporalState(vector<structTime> &_time, vector<structDistance> _v_sD, double _vel)
{
	structTime _sT;
	_time.clear();
	double _dT = 0;
	for (size_t i= 0 ; i < _v_sD.size(); i++){
		_sT.id_from = i;
		_sT.id_to = i+1;
		_dT = (_v_sD[i].dist)/_vel;
		_sT.time = _dT;
		_time.push_back(_sT);
	}
}

// inline void OptimizerLocalPlanner::getSlopXYZAxes(vector<float> &m_) {m_=v_slopeXYZ;}	

void OptimizerLocalPlanner::writeTemporalDataBeforeOptimization(void){
	//! Save temporal state before optimization
	file_in_time.open (path+"initial_time.txt");
	file_in_velocity.open (path+"initial_velocity.txt");
	// file_in_difftime.open (path+"initial_difftime.txt");
	file_in_acceleration.open (path+"initial_acceleration.txt");
	
	double _sum_dist = 0.0;
	double _sum_dT = 0.0;
	for (size_t i = 0; i < vec_time_init.size(); i++){
		_sum_dist = _sum_dist + vec_dist_init[i].dist;
		_sum_dT = _sum_dT + vec_time_init[i].time;	
		file_in_time << setprecision(6) << _sum_dist << ";" << _sum_dT << endl;
		file_in_velocity  << setprecision(6) << _sum_dist << ";" << velocity << endl;
		// file_in_difftime << setprecision(6) << _sum_dist << ";" << _dT << endl;
		if (i > 0)
			file_in_acceleration << setprecision(6) << _sum_dist << ";" << acceleration << endl;
	}
	file_in_time.close();	
	file_in_velocity.close();	
	// file_in_difftime.close();	
	file_in_acceleration.close();
}

void OptimizerLocalPlanner::writeTemporalDataAfterOptimization(auto _s)
{
	//! Save Temporal Data Optimized in File.txt 
	file_out_time.open (path+"optimized_time.txt");
	file_out_velocity.open (path+"optimized_velocity.txt");
	// file_out_difftime.open (path+"optimized_difftime.txt");
	file_out_acceleration.open (path+"optimized_acceleration.txt");
	double sum_dispos = 0.0;
	double sum_diffTime = 0.0;
	double new_vel_ = 0.0;
	for (size_t i=0; i < _s -1; ++i)
	{
		g2o::VertexTimeDiff *difftime = dynamic_cast<g2o::VertexTimeDiff*>(optimizer.vertex(i+_s));
		vec_time_opt.push_back(difftime->estimate());
		double dist_ = (vec_pose_opt[i].position - vec_pose_opt[i+1].position).norm();
		vec_dist_opt.push_back(dist_);
		sum_dispos = dist_ + sum_dispos;
		sum_diffTime = difftime->estimate()+ sum_diffTime;
		new_vel_ = dist_ / difftime->estimate();
		vec_vel_opt.push_back(new_vel_);
		file_out_time << setprecision(6) << sum_dispos << ";" << sum_diffTime << endl;
		file_out_velocity << setprecision(6) << sum_dispos << ";" << new_vel_ << endl;
		// 	file_out_difftime << setprecision(6) << sum_dispos << ";" << difftime->estimate() << endl;
	}
	double sumdis = 0.0;
	for (size_t i = 0; i < _s-2; i++)
	{
		g2o::VertexTimeDiff *difftime1 = dynamic_cast<g2o::VertexTimeDiff*>(optimizer.vertex(i+_s));
		g2o::VertexTimeDiff *difftime2 = dynamic_cast<g2o::VertexTimeDiff*>(optimizer.vertex(i+_s+1));

		double distance1 = (vec_pose_opt[i+1].position - vec_pose_opt[i].position).norm();	
		double distance2 = (vec_pose_opt[i+2].position - vec_pose_opt[i+1].position).norm();
		if (i==0)
			sumdis = distance1;
		sumdis = sumdis + distance2;
		double diffTime = difftime1->estimate()+ difftime2->estimate();
		double velocity1 = distance1 / difftime1->estimate();
		double velocity2 = distance2 / difftime2->estimate();
		double acceleration_ = (velocity2-velocity1)/diffTime;
		vec_acc_opt.push_back(acceleration_);
		file_out_acceleration << setprecision(6) << sumdis << ";" << acceleration_ << endl;
	}
	file_out_time.close();
	file_out_velocity.close();
	// file_out_difftime.close();
	file_out_acceleration.close();
}

void OptimizerLocalPlanner::setInitialLengthCatenaryAndPosUGV(std::vector <double> &_vector, auto _s)
{
	tfListener();

	_vector.clear();
	for (size_t i = 0; i < _s; i++){
		double _dist_X_c_v = (ugv_pos_catenary.x - new_path_from_global[i].x());
		double _dist_Y_c_v = (ugv_pos_catenary.y - new_path_from_global[i].y());
		double _dist_Z_c_v = (ugv_pos_catenary.z - new_path_from_global[i].z());
		
		double _dist_c_v = sqrt(_dist_X_c_v * _dist_X_c_v + _dist_Y_c_v * _dist_Y_c_v + _dist_Z_c_v * _dist_Z_c_v)*initial_multiplicative_factor_length_catenary;  
		
		_vector.push_back(_dist_c_v);
	}
}

void OptimizerLocalPlanner::preComputeLengthCatenary(std::vector <double> &_vector, auto _s)
{
	std::vector<geometry_msgs::Point> _pre_points_catenary;
	Eigen::Vector3d _obstacles_near_catenary;
	Eigen::Vector3d _p_cat;
	struct catenaryStates{
		int _vertex;
		double _initial;
		double _final;
		int _n_collision;
		double _mf;
	};
	vector <catenaryStates> _v_cat_states;
	catenaryStates _cat_states;

	double _mF = initial_multiplicative_factor_length_catenary;
	double _best_length= 0.0;
	int _best_n_collision= 10000;
	bool _state_collision_z ;
	int _n_collision;
	
	tfListener();
	double _bound_z_negative = z_constraint;
	_vector.clear();

	size_t i=0;
	_v_cat_states.clear();
	int change_vertex = -1;

	while (i < _s){
		double _dist_X_c_v = (ugv_pos_catenary.x - new_path_from_global[i].x());
		double _dist_Y_c_v = (ugv_pos_catenary.y - new_path_from_global[i].y());
		double _dist_Z_c_v = (ugv_pos_catenary.z - new_path_from_global[i].z());
		double _dist_c_v = sqrt(_dist_X_c_v * _dist_X_c_v + _dist_Y_c_v * _dist_Y_c_v + _dist_Z_c_v * _dist_Z_c_v);
		double _length = _dist_c_v * _mF;

		if (change_vertex != i){
			_cat_states._vertex = i;
			_cat_states._initial = _length;
			_cat_states._final = -1;
			_cat_states._n_collision = -1;
			_cat_states._mf = -1;
			change_vertex = i;
		}

		_state_collision_z = false;
		_n_collision = 0;
		
		bisectionCatenary bsC;
		bsC.setNumberPointsCatenary(_length*10.0);
		bsC.setFactorBisection(bound_bisection_a,bound_bisection_b);
		bsC.configBisection(_length,ugv_pos_catenary.x,ugv_pos_catenary.y,ugv_pos_catenary.z,new_path_from_global[i].x(),new_path_from_global[i].y(),new_path_from_global[i].z(),i,"pre_compute_cat");
		_pre_points_catenary.clear();
		bsC.getPointCatenary3D(_pre_points_catenary);

		int _n_points_cat_dis = ceil(1.5*ceil(_length)); // parameter to ignore collsion points in the begining and in the end of catenary
		if (_n_points_cat_dis < 5)
			_n_points_cat_dis = 5;

		for (size_t j = 0 ; j <_pre_points_catenary.size() ; j++){
			if (j > _n_points_cat_dis ){
				if(_pre_points_catenary[j].z < _bound_z_negative){
					ROS_ERROR("Pre Compute Catenary with z_value(%f) < z_bound(%f): IMPOSIBLE continue increasing length=[%f] for vertex=[%lu]",_pre_points_catenary[j].z,_bound_z_negative,_length,i);
					_state_collision_z= true;
					break;
				}
				
				_p_cat.x()= _pre_points_catenary[j].x;
				_p_cat.y()= _pre_points_catenary[j].y;
				_p_cat.z()= _pre_points_catenary[j].z;

				_obstacles_near_catenary = nn_.nearestObstacleVertex(nn_.kdtree, _p_cat, nn_.obs_points);
				double _d_cat_obs = (_p_cat-_obstacles_near_catenary).norm();

				if (_d_cat_obs < distance_catenary_obstacle){
					_n_collision++;
				}
			}
		}

		if (_n_collision > 0 || _state_collision_z){
			_mF = _mF + 0.01;

			if (_n_collision < _best_n_collision && !_state_collision_z){
				_best_length = _length;
				_best_n_collision = _n_collision;	
			}
			if(_state_collision_z){
					_cat_states._final = _best_length;
				    _cat_states._mf = _mF;
					_cat_states._n_collision = _best_n_collision;
					_v_cat_states.push_back(_cat_states);
					_mF = initial_multiplicative_factor_length_catenary; 
					_vector.push_back(_best_length);
					i++;
					_best_n_collision = 10000;
			}
		}
		else{
			_cat_states._final = _length;
			_cat_states._mf = _mF;
			_cat_states._n_collision = _n_collision;
			_v_cat_states.push_back(_cat_states);
			_mF = initial_multiplicative_factor_length_catenary; 
			_vector.push_back(_length);
			i++;
			_best_n_collision = 10000;
		}
	}
	printf("---------------------------------------------------------------------------------------------\n");
	for(size_t i = 0; i < _v_cat_states.size(); i++){
		printf("preComputeLengthCatenary: Got it PRE LENGTH catenary = [vertex=%i , initial=%f , final=%f , n_coll=%i , mf=%f]\n",
		_v_cat_states[i]._vertex,_v_cat_states[i]._initial,_v_cat_states[i]._final,_v_cat_states[i]._n_collision,_v_cat_states[i]._mf);
	}
}

void OptimizerLocalPlanner::tfListener(){

	tf::StampedTransform transform;
    listener.lookupTransform("/map", "/reel_base_link", ros::Time(0), transform);

	ugv_pos_catenary.x = transform.getOrigin().x();
	ugv_pos_catenary.y = transform.getOrigin().y();
	ugv_pos_catenary.z = transform.getOrigin().z();
}


void OptimizerLocalPlanner::checkObstaclesBetweenCatenaries(std::vector <double> _vectorIN,std::vector <double> &_vectorOUT, auto _s)
{
	int _num_points;
	double _length1,_length2;
	bool obst_between_catenaries=false;
	std::vector<geometry_msgs::Point> _pre_points_catenary1;
	std::vector<geometry_msgs::Point> _pre_points_catenary2;
	std::vector<geometry_msgs::Point> _v_factor1, _v_factor2;
	std::vector<Eigen::Vector3d> _vector_points_line;
	bisectionCatenary bsC1, bsC2;

	_vectorOUT.clear();
	int i = 0;
	int _count_obst;
	bool _z_collision;
	double _mf = 1.0;
	double best_count_obs = 10000;
	double best_length2 = 0.0;

	_vectorOUT.push_back(_vectorIN[0]);
	while( i < _s-1){
		if(_vectorIN[i] ==_vectorOUT[i])
			_length1 = _vectorIN[i];
		else
			_length1 = _vectorOUT[i];
		if (!obst_between_catenaries)
			_length2 = _vectorIN[i+1];
		else
			_length2 = _vectorIN[i+1] * _mf;
		
		bsC1.setNumberPointsCatenary(_length1*10.0);
		bsC1.setFactorBisection(bound_bisection_a,bound_bisection_b);
		bsC1.configBisection(_length1,ugv_pos_catenary.x,ugv_pos_catenary.y,ugv_pos_catenary.z,new_path_from_global[i].x(),new_path_from_global[i].y(),new_path_from_global[i].z(),i,"pre_compute_cat1");
		_pre_points_catenary1.clear();
		bsC1.getPointCatenary3D(_pre_points_catenary1);
		
		bsC2.setNumberPointsCatenary(_length2*10.0);
		bsC2.setFactorBisection(bound_bisection_a,bound_bisection_b);
		bsC2.configBisection(_length2,ugv_pos_catenary.x,ugv_pos_catenary.y,ugv_pos_catenary.z,new_path_from_global[i+1].x(),new_path_from_global[i+1].y(),new_path_from_global[i+1].z(),i,"pre_compute_cat2");
		_pre_points_catenary2.clear();
		bsC2.getPointCatenary3D(_pre_points_catenary2);

		double n_points_cat_dis1 = ceil(1.5*ceil(_length1)); // parameter to ignore collsion points in the begining and in the end of catenary
		if (n_points_cat_dis1 < 5)
			n_points_cat_dis1 = 5;
		double n_points_cat_dis2 = ceil(1.5*ceil(_length2)); // parameter to ignore collsion points in the begining and in the end of catenary
		if (n_points_cat_dis2 < 5)
			n_points_cat_dis2 = 5;			

		double rate;
		string equal_status;
		rate = (double)_pre_points_catenary1.size()/(double)_pre_points_catenary2.size();	

		_count_obst = 0;
		_z_collision = false;
		for (size_t j=0; j< _pre_points_catenary2.size(); j++){
			int id_c2_to_c1; 
			id_c2_to_c1 = ceil((j+1)*rate) -1 ;
			if (_pre_points_catenary1.size() <= _pre_points_catenary2.size())
				equal_status ="catenary1.size() <= catenary2.size()";
			else
				equal_status ="catenary2.size() < catenary1.size()";

			if ( j > n_points_cat_dis2  && j < _pre_points_catenary2.size()-n_points_cat_dis2/2 ){
				Eigen::Vector3d _pc1, _pc2;
				_pc1.x() = _pre_points_catenary1[id_c2_to_c1].x;
				_pc1.y() = _pre_points_catenary1[id_c2_to_c1].y;
				_pc1.z() = _pre_points_catenary1[id_c2_to_c1].z;
				_pc2.x() = _pre_points_catenary2[j].x;
				_pc2.y() = _pre_points_catenary2[j].y;
				_pc2.z() = _pre_points_catenary2[j].z;
				if (_pc1.z() < 0.1 ){
					ROS_ERROR("Warning: Not Posible to get feasible lenght for catenary1 %i , best_length2=[%f]",i,best_length2);
					i++;
					_z_collision = true;
					_mf = 1.0;
					_vectorOUT.push_back(best_length2);
					break;
				}
				if ( _pc2.z()<0.1){
					ROS_ERROR("Warning: Not Posible to get feasible lenght for catenary2 %i , best_length2=[%f]",i+1,best_length2);
					i++;
					_z_collision = true;
					_mf = 1.0;
					_vectorOUT.push_back(best_length2);
					break;
				}

				double dist_pc2_pc1 = (_pc2 - _pc1).norm();
				_num_points = (int)(ceil(dist_pc2_pc1 * 10.0));
				_vector_points_line.clear();
				straightTrajectoryVertices(_pc1.x(),_pc1.y(),_pc1.z(),_pc2.x(),_pc2.y(),_pc2.z(),_num_points,_vector_points_line);
				for (size_t k =0; k <_vector_points_line.size(); k++){
					Eigen::Vector3d _near = nn_.nearestObstacleVertex(nn_.kdtree, _vector_points_line[k], nn_.obs_points);
					double dist_straight_line_obstacle = (_vector_points_line[k] -_near).norm();
					double _radius_cat_straight = 0.1;
					if (dist_straight_line_obstacle < _radius_cat_straight){
						_count_obst++;
					}
				}
			}
		}

		if(_count_obst > 0){
			_mf = _mf + 0.01;
			obst_between_catenaries = true ;
			best_length2 = _length2;
		}
		else if(!_z_collision){
			_mf = 1.0;
			obst_between_catenaries = false;
			_vectorOUT.push_back(_length2);
			i++;
		}
		markerPoints(_catenary_marker ,_pre_points_catenary1,i-1,_s-2);
		markerPoints(_catenary_marker ,_pre_points_catenary2,i,_s-2);
	}
	
	printf("---------------------------------------------------------------------------------------------\n");
	for(size_t i = 0; i < _vectorOUT.size(); i++){
		structLengthCatenary l_;
		l_.id = i;
		l_.length = _vectorOUT[i];
		vec_len_cat_init.push_back(l_);
		printf("checkObstaclesBetweenCatenaries: Got it LENGTH catenary = [vertex=%lu , initial=%f , final=%f ]\n",i,_vectorIN[i],_vectorOUT[i]);
	}
}

void OptimizerLocalPlanner::straightTrajectoryVertices(double x1, double y1, double z1, double x2, double y2, double z2, int _n_v_u, std::vector<Eigen::Vector3d> &_v)
{
	double _x, _y, _z;
	double _d= sqrt(pow(x2-x1,2)+pow(y2-y1,2)+pow(z2-z1,2));
	double distance_xy = sqrt(pow(x2-x1,2)+pow(y2-y1,2));
	double distance_xz = sqrt(pow(x2-x1,2)+pow(z2-z1,2));
	if (distance_xy < 0.00001)
		distance_xy = 0.00001;
	if (distance_xz < 0.00001)
		distance_xz = 0.00001;

	double interval_xy = distance_xy/_n_v_u;		// Size of the interval between points in axes xy
	double interval_xz = distance_xz/_n_v_u;		// Size of the interval between points in axes xz
	for(int i = 1 ; i< _n_v_u+1 ; i++)
	{
		_x = x1 + i*interval_xy * ((x2-x1)/distance_xy);
		_y = y1 + i*interval_xy * ((y2-y1)/distance_xy);
		_z = z1 + i*interval_xz * ((z2-z1)/distance_xz);
	
		_v.push_back(Eigen::Vector3d(_x,_y,_z));
	}
}

	void OptimizerLocalPlanner::markerPoints(visualization_msgs::MarkerArray _marker, std::vector<geometry_msgs::Point> _vector, int _suffix, int _n_v)
	{
		std::string string_marker;
		std::string ns_marker;

		double c_color1 = (_suffix / (double)_n_v)* 0.5;
		double c_color2 = (_suffix / (double)_n_v)* 0.5;
	  	
		string_marker = std::to_string(_suffix);
		ns_marker = "catenary_"+string_marker;

		_marker.markers.resize(_vector.size());
		
		for (size_t i = 0; i < _vector.size(); ++i){
			_marker.markers[i].header.frame_id = "/map";
			_marker.markers[i].header.stamp = ros::Time::now();
			_marker.markers[i].ns = ns_marker;
			_marker.markers[i].id = i+1;
			_marker.markers[i].action = visualization_msgs::Marker::ADD;
			if (i % 5 == 0)
				_marker.markers[i].type = visualization_msgs::Marker::CUBE;
			else
				_marker.markers[i].type = visualization_msgs::Marker::SPHERE;
			_marker.markers[i].lifetime = ros::Duration(400);
			_marker.markers[i].pose.position.x = _vector[i].x; 
			_marker.markers[i].pose.position.y = _vector[i].y; 
			_marker.markers[i].pose.position.z = _vector[i].z;

			_marker.markers[i].pose.orientation.x = 0.0;
			_marker.markers[i].pose.orientation.y = 0.0;
			_marker.markers[i].pose.orientation.z = 0.0;
			_marker.markers[i].pose.orientation.w = 1.0;
			_marker.markers[i].scale.x = 0.06;
			_marker.markers[i].scale.y = 0.06;
			_marker.markers[i].scale.z = 0.06;
			_marker.markers[i].color.a = 1.0;
			_marker.markers[i].color.r = 0.9;
			_marker.markers[i].color.g = c_color1;
			_marker.markers[i].color.b = 0.0;
		}	
		catenary_marker_pub_.publish(_marker);
	}

	void OptimizerLocalPlanner::clearMarkers(visualization_msgs::MarkerArray _marker,auto _s)
	{
		_marker.markers.clear();
		_marker.markers.resize(_s);

		for (auto i = 0 ; i < _s; i++){
			_marker.markers[i].action = visualization_msgs::Marker::DELETEALL;
		}
		catenary_marker_pub_.publish(_marker);
	}


