/*
Optimizer Local Planner based in g2o for Marsupial Robotics COnfiguration 
Simon Martinez Rozas, 2020 
Service Robotics Lab, University Pablo de Olavide , Seville, Spain 
 */

#include "marsupial_optimizer/optimizer_local_planner.h"


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
	nh->param<double>("w_gamma", w_gamma,0.1);
	nh->param<double>("w_delta", w_delta,0.1);
	nh->param<double>("w_epsilon", w_epsilon,0.1);
	nh->param<double>("w_zeta", w_zeta,0.1);
	nh->param<double>("w_eta", w_eta,0.1);
	nh->param<double>("w_lambda", w_lambda,0.1);

  	nh->param<double>("initial_multiplicative_factor_length_catenary", initial_multiplicative_factor_length_catenary,1.0); //Can't be lower than 1.0 or "catenary < distance between UGV and state" 	

	nh->param<int>("n_iter_opt", n_iter_opt,200);
	nh->param<double>("distance_obstacle", distance_obstacle,2.0);
	nh->param<double>("initial_velocity_ugv", initial_velocity_ugv,1.0);
	nh->param<double>("initial_velocity_uav", initial_velocity_uav,1.0);
	nh->param<double>("initial_acceleration_ugv", initial_acceleration_ugv,0.0);
	nh->param<double>("initial_acceleration_uav", initial_acceleration_uav,0.0);
	nh->param<double>("angle_min_traj", angle_min_traj, M_PI / 15.0);
	nh->param<double>("distance_catenary_obstacle", distance_catenary_obstacle, 0.1);
	nh->param<double>("dynamic_catenary", dynamic_catenary, 0.5);
	nh->param<double>("min_distance_add_new_point", min_distance_add_new_point, 0.5);

	nh->param<double>("pos_reel_x", pos_reel_x, 0.4);
    nh->param<double>("pos_reel_y", pos_reel_y, 0.0);
    nh->param<double>("pos_reel_z", pos_reel_z, 0.22);

	nh->param<double>("z_constraint", z_constraint,0.0);
	nh->param<double>("length_tether_max", length_tether_max,20.0);

	nh->param<bool>("verbose_optimizer",verbose_optimizer, true);

	nh->param<bool>("write_data_for_analysis",write_data_for_analysis, false);
	nh->param("path", path, (std::string) "/home/simon/");
	nh->param("name_output_file", name_output_file, (std::string) "optimization_test");
	nh->param<int>("scenario_number", scenario_number, 1);
	nh->param<int>("num_pos_initial", num_pos_initial, 1);
	nh->param<int>("num_goal", num_goal, 0);
		
	nh->param("debug", debug, (bool)0);
 	 nh->param("show_config", showConfig, (bool)0);

	ROS_INFO_COND(showConfig, PRINTF_GREEN "Optimizer Local Planner 3D Node Configuration:\n");
	// ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Trajectory Position Increments = [%.2f], Tolerance: [%.2f]", traj_dxy_max, traj_pos_tol);
	// ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Robot base frame: %s, World frame: %s", uav_base_frame.c_str(), world_frame.c_str());
	// ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Workspace:\t X:[%.2f, %.2f]\t Y:[%.2f, %.2f]\t Z: [%.2f, %.2f]", ws_x_max, ws_x_min, ws_y_max, ws_y_min, ws_z_max, ws_z_min);
	// ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Map Resolution: %.2f\t Map H inflaction: %.2f\t Map V Inflaction: %.2f", map_resolution, map_h_inflaction, map_v_inflaction);
	// ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Z weight cost: %.2f\t Z not inflate: %.2f", z_weight_cost, z_not_inflate);
		
	pos_reel_ugv.x = pos_reel_x;
	pos_reel_ugv.y = pos_reel_y;
	pos_reel_ugv.z = pos_reel_z;

	initializeSubscribers();
	initializePublishers();
	setupOptimizer();
	cleanVectors();
	// ROS_INFO(PRINTF_BLUE"alpha=[%f] beta=[%f] gamma=[%f] delta=[%f] epsilon=[%f] zeta=[%f] eta=[%f] lambda=[%f]",w_alpha,w_beta,w_gamma,w_delta,w_epsilon,w_zeta,w_eta,w_lambda);
	ROS_INFO(PRINTF_BLUE"Optimizer_Local_Planner: Parameters loaded, ready for Optimization Process. Waiting for Action!!");
	// traj_marker_uav_pub_.publish(points_uav_marker);
	// traj_marker_uav_pub_.publish(lines_uav_marker);
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
    
	octomap_ws_sub_ = nh->subscribe( "/octomap_point_cloud_centers", 1,  &OptimizerLocalPlanner::readOctomapCallback, this);
    point_cloud_ugv_traversability_sub_ = nh->subscribe( "/region_growing_traversability_pc_map", 1,  &OptimizerLocalPlanner::readPointCloudTraversabilityUGVCallback, this);
    point_cloud_ugv_obstacles_sub_ = nh->subscribe( "/region_growing_obstacles_pc_map", 1,  &OptimizerLocalPlanner::readPointCloudObstaclesUGVCallback, this);
    
	ROS_INFO(PRINTF_BLUE"Optimizer_Local_Planner: Subscribers Initialized");
}

void OptimizerLocalPlanner::initializePublishers()
{
  	traj_marker_ugv_pub_ = nh->advertise<visualization_msgs::MarkerArray>("opt_trajectory_ugv_marker", 2);
  	traj_marker_uav_pub_ = nh->advertise<visualization_msgs::MarkerArray>("opt_trajectory_uav_marker", 2);
	catenary_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("catenary_marker", 100);

  	ROS_INFO(PRINTF_BLUE"Optimizer_Local_Planner: Publishers Initialized");
}

void OptimizerLocalPlanner::resetFlags()
{
  	mapReceived = false;
}

void OptimizerLocalPlanner::cleanVectors()
{
	new_path_ugv.clear();
	new_path_uav.clear();	
	vec_rot_ugv.clear(); 
	vec_rot_uav.clear();
	vec_len_cat_init.clear();
	
	vec_pose_init_ugv.clear();
	vec_pose_init_uav.clear();
	
	vec_dist_init_ugv.clear();
	vec_dist_init_uav.clear();

	vec_time_init_ugv.clear();
	vec_time_init_uav.clear();
}

void OptimizerLocalPlanner::configServices()
{
    execute_path_srv_ptr.reset(new ExecutePathServer(*nh, "/Execute_Plan", false));
    execute_path_srv_ptr->registerGoalCallback(boost::bind(&OptimizerLocalPlanner::executeOptimizerPathGoalCB, this));
    execute_path_srv_ptr->registerPreemptCallback(boost::bind(&OptimizerLocalPlanner::executeOptimizerPathPreemptCB, this));
    execute_path_srv_ptr->start();
}

void OptimizerLocalPlanner::setupOptimizer()
{
  options.max_num_iterations = n_iter_opt;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;
}

void OptimizerLocalPlanner::dynRecCb(marsupial_optimizer::OptimizationParamsConfig &config, uint32_t level)
{
    // this->cost_weight = config.cost_weight;
    // this->lof_distance = config.lof_distance;
    // this->occ_threshold = config.occ_threshold;
    // this->goal_weight = config.goal_weight;
}

void OptimizerLocalPlanner::readOctomapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	nn_uav.setInput(*msg);
  	ROS_INFO_COND(debug, PRINTF_BLUE "Local Planner: Received Point Cloud for KDTree UAV");

}

void OptimizerLocalPlanner::readPointCloudTraversabilityUGVCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	nn_ugv_tra.setInput(*msg);
  	ROS_INFO_COND(debug, PRINTF_BLUE "Local Planner: Received Point Cloud for KDTree traversability UGV");
}

void OptimizerLocalPlanner::readPointCloudObstaclesUGVCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	nn_ugv_obs.setInput(*msg);
  	ROS_INFO_COND(debug, PRINTF_BLUE "Local Planner: Received Point Cloud for KDTree obstacles UGV");
}

void OptimizerLocalPlanner::collisionMapCallBack(const octomap_msgs::OctomapConstPtr &msg)
{
    mapReceived = true;
}

void OptimizerLocalPlanner::executeOptimizerPathPreemptCB()
{
    ROS_INFO_COND(debug, "Goal Preempted");
    execute_path_srv_ptr->setPreempted(); // set the action state to preempted

    resetFlags();
    mp_.clearMarkersPointLines(points_ugv_marker, lines_ugv_marker, traj_marker_ugv_pub_, 0);
    mp_.clearMarkersPointLines(points_uav_marker, lines_uav_marker, traj_marker_uav_pub_, 0);
}


void OptimizerLocalPlanner::executeOptimizerPathGoalCB()
{
  	ROS_INFO_COND(debug, PRINTF_GREEN "Optimizer Local Planner Goal received in action server mode");
  	auto path_shared_ptr = execute_path_srv_ptr->acceptNewGoal();
  	globalTrajectory = path_shared_ptr->path;
	vec_len_cat_init = path_shared_ptr->length_catenary; 
	printf("vec_len_cat_init.size=[%lu]\n",vec_len_cat_init.size());

	for (size_t i = 0; i < globalTrajectory.points.size(); i++){
		printf("point=[%lu/%lu] GlobalTrayectoryPointTranslation=[%f %f %f / %f %f %f / %f]\n",i+1, globalTrajectory.points.size(),
		globalTrajectory.points.at(i).transforms[0].translation.x, globalTrajectory.points.at(i).transforms[0].translation.y, globalTrajectory.points.at(i).transforms[0].translation.z,
		globalTrajectory.points.at(i).transforms[1].translation.x, globalTrajectory.points.at(i).transforms[1].translation.y, globalTrajectory.points.at(i).transforms[1].translation.z,
		vec_len_cat_init[i]);
	}
		
	getPointsFromGlobalPath(globalTrajectory, new_path_ugv, new_path_uav);
    auto size = new_path_uav.size();
    ROS_INFO_COND(debug, PRINTF_GREEN "Number of Vertices to optimize = [%lu] = size",size);

	printf("ugv_pos_reel_catenary=[%f %f %f]\n",ugv_pos_catenary.x,ugv_pos_catenary.y,ugv_pos_catenary.z);
	mp_.clearMarkers(catenary_marker, 150, catenary_marker_pub_);
	// ros::Duration(2.0).sleep();

  	mp_.clearMarkersPointLines(points_ugv_marker, lines_ugv_marker,traj_marker_ugv_pub_,size);
  	mp_.clearMarkersPointLines(points_uav_marker, lines_uav_marker,traj_marker_uav_pub_,size);
	calculateDistanceVertices(vec_dist_init_ugv, vec_dist_init_uav);
 	getTemporalState(vec_time_init_ugv, vec_time_init_uav);

	dm_.initDataManagement(path, name_output_file, ugv_pos_catenary, scenario_number, num_pos_initial, num_goal, initial_velocity_ugv, initial_velocity_uav, initial_acceleration_ugv, 
						   initial_acceleration_uav, vec_pose_init_ugv, vec_pose_init_uav, vec_len_cat_init);			   
	dm_.writeTemporalDataBeforeOptimization(vec_dist_init_ugv, vec_dist_init_uav, vec_time_init_ugv, vec_time_init_uav);

	std::cout << std::endl <<  "==================================================="  << std::endl << "Preparing to execute Optimization: Creating Parameter Blocks!!" << std::endl;

  	Problem problem;

	//Set Parameter Blocks from path global information
	statesPosUGV.clear();
	statesPosUAV.clear();
	statesRot.clear();
	statesTime.clear();
	statesLength.clear();
	printf("[_]states.parameter=[pos_x  pos_y  pos_z  rot_x  rot_y  rot_z  rot_w  time  length_]\n");
	for (size_t i = 0 ; i < new_path_uav.size(); i ++){
		parameterBlockPos parameter_block_pos_ugv;
		parameterBlockPos parameter_block_pos_uav;
		parameterBlockRot parameter_block_rot;
		parameterBlockTime parameter_block_time;
		parameterBlockLength parameter_block_length;
		//Position Parameters for UGV and UAV
		parameter_block_pos_ugv.parameter[0] = i; //id parameter
		parameter_block_pos_ugv.parameter[1] = vec_pose_init_ugv[i].x();
		parameter_block_pos_ugv.parameter[2] = vec_pose_init_ugv[i].y(); 
		parameter_block_pos_ugv.parameter[3] = vec_pose_init_ugv[i].z(); 
		parameter_block_pos_uav.parameter[0] = i; //id parameter
		parameter_block_pos_uav.parameter[1] = vec_pose_init_uav[i].x();
		parameter_block_pos_uav.parameter[2] = vec_pose_init_uav[i].y(); 
		parameter_block_pos_uav.parameter[3] = vec_pose_init_uav[i].z(); 
		//Rotation Parameters for UGV and UAV
		parameter_block_rot.parameter[0] = i; //id parameter
		parameter_block_rot.parameter[1] = vec_rot_ugv[i].x;
		parameter_block_rot.parameter[2] = vec_rot_ugv[i].y; 
		parameter_block_rot.parameter[3] = vec_rot_ugv[i].z; 
		parameter_block_rot.parameter[4] = vec_rot_ugv[i].w;
		parameter_block_rot.parameter[5] = vec_rot_uav[i].x;
		parameter_block_rot.parameter[6] = vec_rot_uav[i].y; 
		parameter_block_rot.parameter[7] = vec_rot_uav[i].z; 
		parameter_block_rot.parameter[8] = vec_rot_uav[i].w; 
		//Time Parameters for UGV and UAV
		parameter_block_time.parameter[0] = i; //id parameter
		parameter_block_time.parameter[1] = vec_time_init_ugv[i];
		parameter_block_time.parameter[2] = vec_time_init_uav[i];
		//Length Cable Parameters for UGV and UAV
		parameter_block_length.parameter[0] = i; //id parameter
		parameter_block_length.parameter[1] = vec_len_cat_init[i];

		statesPosUGV.push_back(parameter_block_pos_ugv);
		statesPosUAV.push_back(parameter_block_pos_uav);
		statesRot.push_back(parameter_block_rot);
		statesTime.push_back(parameter_block_time);
		statesLength.push_back(parameter_block_length);
		printf("[%lu]states.parameterUGV=[%f %f %f / %f %f %f %f/ %f / %f]\n",i,parameter_block_pos_ugv.parameter[1],parameter_block_pos_ugv.parameter[2],parameter_block_pos_ugv.parameter[3],
															parameter_block_rot.parameter[1],parameter_block_rot.parameter[2],parameter_block_rot.parameter[3],parameter_block_rot.parameter[4],
															parameter_block_time.parameter[1],parameter_block_length.parameter[1]);
		printf("[%lu]states.parameterUAV=[%f %f %f / %f %f %f %f/ %f / %f]\n",i,parameter_block_pos_uav.parameter[1],parameter_block_pos_uav.parameter[2],parameter_block_pos_uav.parameter[3],
															parameter_block_rot.parameter[5],parameter_block_rot.parameter[6],parameter_block_rot.parameter[7],parameter_block_rot.parameter[8],
															parameter_block_time.parameter[2],parameter_block_length.parameter[1]);
	}
    
	/*** Cost Function I-i : UAV Equidistance constrain ***/
	for (int i = 0; i <  statesPosUAV.size() - 1 ; ++i) {
		CostFunction* cost_function1_1  = new AutoDiffCostFunction<EquiDistanceFunctorUAV, 1, 4, 4>(new EquiDistanceFunctorUAV(w_alpha*10.0, initial_distance_states_ugv, initial_distance_states_uav)); 
		problem.AddResidualBlock(cost_function1_1, NULL, statesPosUAV[i].parameter, statesPosUAV[i+1].parameter);
		if (i == 0)
			problem.SetParameterBlockConstant(statesPosUAV[i].parameter);
		if (i == (statesPosUAV.size() - 2))
			problem.SetParameterBlockConstant(statesPosUAV[i+1].parameter);
		problem.SetParameterLowerBound(statesPosUAV[i].parameter, 3, 0.0);
		problem.SetParameterUpperBound(statesPosUAV[i].parameter, 3, 20.0);
	}

	/*** Cost Function I-ii : UGV Equidistance constrain ***/
	for (int i = 0; i <  statesPosUGV.size() - 1 ; ++i) {
		CostFunction* cost_function1_2  = new AutoDiffCostFunction<EquiDistanceFunctorUGV, 1, 4, 4>(new EquiDistanceFunctorUGV(w_alpha*10.0, initial_distance_states_ugv, initial_distance_states_uav)); 
		problem.AddResidualBlock(cost_function1_2, NULL, statesPosUGV[i].parameter, statesPosUGV[i+1].parameter);
		if (i == 0)
			problem.SetParameterBlockConstant(statesPosUGV[i].parameter);
	}

	/*** Cost Function II-i : Obstacles constrain UAV***/
	for (int i = 0; i < statesPosUAV.size(); ++i) {
    	CostFunction* cost_function2_1  = new AutoDiffCostFunction<ObstacleDistanceFunctorUAV::ObstaclesFunctor, 1, 4>
											(new ObstacleDistanceFunctorUAV::ObstaclesFunctor(w_beta*10.0, distance_obstacle, nn_uav.kdtree, nn_uav.obs_points)); 
    	problem.AddResidualBlock(cost_function2_1, NULL, statesPosUAV[i].parameter);
  	}

	/*** Cost Function II-ii : Obstacles constrain UGV***/
	for (int i = 0; i < statesPosUGV.size(); ++i) {
    	CostFunction* cost_function2_2  = new AutoDiffCostFunction<ObstacleDistanceFunctorUGV::ObstaclesFunctor, 1, 4>
											(new ObstacleDistanceFunctorUGV::ObstaclesFunctor(w_beta*10.0, distance_obstacle, nn_ugv_obs.kdtree, nn_ugv_obs.obs_points)); 
    	problem.AddResidualBlock(cost_function2_2, NULL, statesPosUGV[i].parameter);
  	}
	
	// /*** Cost Function II-iii : Traversability constrain UGV***/
	for (int i = 0; i < statesPosUGV.size(); ++i) {
    	CostFunction* cost_function2_3  = new AutoDiffCostFunction<TraversabilityDistanceFunctorUGV::TraversabilityFunctor, 1, 4>
											(new TraversabilityDistanceFunctorUGV::TraversabilityFunctor(w_beta*10.0, nn_ugv_tra.kdtree, nn_ugv_tra.obs_points)); 
    	problem.AddResidualBlock(cost_function2_3, NULL, statesPosUGV[i].parameter);
  	}

	/*** Cost Function III-i : UAV Kinematic constrain ***/
	for (int i = 0; i < statesPosUAV.size() - 2; ++i) {
		CostFunction* cost_function3_1  = new AutoDiffCostFunction<KinematicsFunctorUAV, 1, 4, 4, 4>(new KinematicsFunctorUAV(w_gamma, angle_min_traj)); 
		problem.AddResidualBlock(cost_function3_1, NULL, statesPosUAV[i].parameter, statesPosUAV[i+1].parameter, statesPosUAV[i+2].parameter);
	}

	/*** Cost Function III-ii : UGV Kinematic constrain ***/
	for (int i = 0; i < statesPosUGV.size() - 2; ++i) {
		CostFunction* cost_function3_2  = new AutoDiffCostFunction<KinematicsFunctorUGV, 1, 4, 4, 4>(new KinematicsFunctorUGV(w_gamma, angle_min_traj)); 
		problem.AddResidualBlock(cost_function3_2, NULL, statesPosUGV[i].parameter, statesPosUGV[i+1].parameter, statesPosUGV[i+2].parameter);
	}

	/*** Cost Function IV : Time constrain ***/
	for (int i = 0; i < statesTime.size(); ++i) {
		CostFunction* cost_function4  = new AutoDiffCostFunction<TimeFunctor, 2, 3>(new TimeFunctor(w_delta, vec_time_init_ugv[i], vec_time_init_uav[i])); 
		problem.AddResidualBlock(cost_function4, NULL, statesTime[i].parameter);
		if (i == 0)
			problem.SetParameterBlockConstant(statesTime[i].parameter);
		problem.SetParameterLowerBound(statesTime[i].parameter, 1, 0.0);
	}

	/*** Cost Function V : Velocity constrain ***/
	for (int i = 0; i < statesTime.size() - 1; ++i) {
		CostFunction* cost_function5  = new AutoDiffCostFunction<VelocityFunctor, 2, 4, 4, 4, 4, 3>(new VelocityFunctor(w_epsilon, initial_velocity_ugv, initial_velocity_uav)); 
		problem.AddResidualBlock(cost_function5, NULL, 
								statesPosUAV[i].parameter, statesPosUAV[i+1].parameter, 
								statesPosUGV[i].parameter, statesPosUGV[i+1].parameter, 
								statesTime[i+1].parameter);
	}

	/*** Cost Function VI : Acceleration constrain ***/
	for (int i = 0; i < statesTime.size() - 2; ++i) {
		CostFunction* cost_function6  = new AutoDiffCostFunction<AccelerationFunctor, 2, 4, 4, 4, 4, 4, 4, 3, 3>(new AccelerationFunctor(w_zeta, initial_acceleration_ugv, initial_acceleration_uav)); 
		problem.AddResidualBlock(cost_function6, NULL, 
								statesPosUAV[i].parameter, statesPosUAV[i+1].parameter, statesPosUAV[i+2].parameter,
								statesPosUGV[i].parameter, statesPosUGV[i+1].parameter, statesPosUGV[i+2].parameter,  
								statesTime[i+1].parameter, statesTime[i+2].parameter);
	}

	/*** Cost Function VII : Catenary constrain  ***/
	for (int i = 0; i < statesLength.size(); ++i) {
		CostFunction* cost_function7  = new NumericDiffCostFunction<CatenaryFunctor, CENTRAL, 2, 4, 4, 9, 2>
										(new CatenaryFunctor(w_eta * 10.0, distance_catenary_obstacle, vec_len_cat_init[i], nn_uav.kdtree, nn_uav.obs_points, pos_reel_ugv, size, nh)); 
		problem.AddResidualBlock(cost_function7, NULL, statesPosUAV[i].parameter, statesPosUGV[i].parameter, statesRot[i].parameter, statesLength[i].parameter);
		if (i == 0)
			problem.SetParameterBlockConstant(statesLength[i].parameter);
		problem.SetParameterLowerBound(statesLength[i].parameter, 1, 0.0);
		problem.SetParameterUpperBound(statesLength[i].parameter, 1, length_tether_max);
	}

	/*** Cost Function VIII : Dynamic Catenary constrain  ***/
	// for (int i = 0; i < statesLength.size() - 1; ++i) {
	// 	CostFunction* cost_function8  = new AutoDiffCostFunction<DynamicCatenaryFunctor, 1, 2, 2>(new DynamicCatenaryFunctor(w_lambda, dynamic_catenary)); 
	// 	problem.AddResidualBlock(cost_function8, NULL, statesLength[i].parameter, statesLength[i+1].parameter);
	// }

	std::cout <<  "==================================================="  << std::endl << "Optimization  started !!" << std::endl;

	start_time_opt = ros::Time::now();

	ceres::Solve(options, &problem, &summary);
	std::cout << summary.BriefReport() << "\n";
	final_time_opt = ros::Time::now();

	std::cout << std::endl <<"Optimization Proccess Completed !!!" << std::endl << "===================================================" << std::endl << "Saving Temporal data in txt file ..." << std::endl;

	vec_pose_ugv_opt.clear();
	vec_pose_uav_opt.clear();
	vec_time_ugv_opt.clear();
	vec_time_uav_opt.clear();
	vec_len_cat_opt.clear();
	new_path_ugv.clear(); //clear the old path to set the optimizer solution as a new path
	new_path_uav.clear(); //clear the old path to set the optimizer solution as a new path

  	for (size_t i = 0; i < size; i++){
		Eigen::Vector3d position_ugv_ = Eigen::Vector3d(statesPosUGV[i].parameter[1],statesPosUGV[i].parameter[2],statesPosUGV[i].parameter[3]);
		vec_pose_ugv_opt.push_back(position_ugv_);
		Eigen::Vector3d position_uav_ = Eigen::Vector3d(statesPosUAV[i].parameter[1],statesPosUAV[i].parameter[2],statesPosUAV[i].parameter[3]);
		vec_pose_uav_opt.push_back(position_uav_);
		double vLC = statesLength[i].parameter[1];
		vec_len_cat_opt.push_back(vLC);
		vec_time_ugv_opt.push_back(statesTime[i].parameter[1]);
		vec_time_uav_opt.push_back(statesTime[i].parameter[2]);
	}

	mp_.clearMarkers(catenary_marker, 150, catenary_marker_pub_);  //To delete possibles outliers points 
	for(size_t i = 0; i < statesPosUAV.size(); i++){
		std::vector<geometry_msgs::Point> points_catenary_final;
		points_catenary_final.clear();

		geometry_msgs::Point p_reel_=getReelPoint(statesPosUGV[i].parameter[1],statesPosUGV[i].parameter[2],statesPosUGV[i].parameter[3],statesRot[i].parameter[1],statesRot[i].parameter[2],statesRot[i].parameter[3], statesRot[i].parameter[4],pos_reel_ugv);
		CatenarySolver cSX_;
		cSX_.setMaxNumIterations(100);
	  	cSX_.solve(p_reel_.x, p_reel_.y, p_reel_.z, statesPosUAV[i].parameter[1], statesPosUAV[i].parameter[2], statesPosUAV[i].parameter[3], statesLength[i].parameter[1], points_catenary_final);
		mp_.markerPoints(catenary_marker, points_catenary_final, i, size, catenary_marker_pub_);
		double _d_ = sqrt(pow(p_reel_.x -statesPosUAV[i].parameter[1],2) + pow(p_reel_.y - statesPosUAV[i].parameter[2],2) + pow(p_reel_.z-statesPosUAV[i].parameter[3],2));
		printf("points_catenary_final.size()=[%lu] ugv_pos[%lu] = [%f %f %f] , uav_pos[%lu]=[%f %f %f] , l=[%f] d=[%f]\n",points_catenary_final.size(),
																								i,statesPosUGV[i].parameter[1], statesPosUGV[i].parameter[2], statesPosUGV[i].parameter[3],
																								i,statesPosUAV[i].parameter[1], statesPosUAV[i].parameter[2], statesPosUAV[i].parameter[3],
																								statesLength[i].parameter[1],_d_);
	}

	mp_.getMarkerPoints(points_ugv_marker, vec_pose_ugv_opt, "points_ugv_m",3);	// RED: colour_ = 0 ; GREEN : colour_ = 1 ; BLUE: colour_ = 2 ; YELLOW = 3 ; PURPLE = 4;
	mp_.getMarkerLines(lines_ugv_marker, vec_pose_ugv_opt, "lines_ugv_m",3);
	mp_.getMarkerPoints(points_uav_marker, vec_pose_uav_opt, "points_uav_m",1);
	mp_.getMarkerLines(lines_uav_marker, vec_pose_uav_opt, "lines_uav_m",1);
	traj_marker_ugv_pub_.publish(points_ugv_marker);
	traj_marker_ugv_pub_.publish(lines_ugv_marker);
	traj_marker_uav_pub_.publish(points_uav_marker);
	traj_marker_uav_pub_.publish(lines_uav_marker);

	dm_.writeTemporalDataAfterOptimization(size, vec_pose_ugv_opt, vec_pose_uav_opt, vec_time_ugv_opt, vec_time_uav_opt, vec_len_cat_opt);
	std::cout << "...Temporal data saved in txt file." << std::endl << std::endl;
	if (write_data_for_analysis){
		std::cout << "Saving data for analisys in txt file..." << std::endl;
		double opt_compute_time_ = (final_time_opt - start_time_opt).toSec(); //Compute Time Optimization
		dm_.getDataForOptimizerAnalysis(nn_uav.kdtree, nn_uav.obs_points, opt_compute_time_);
		std::cout << "... data for analysis saved in txt file." << std::endl << std::endl;
	}

	cleanVectors();		//Clear vector after optimization and previus the next iteration

	ROS_INFO("Optimizer Local Planner: Goal position successfully achieved");
	ros::Duration(5.0).sleep();
	action_result.arrived = true;
	execute_path_srv_ptr->setSucceeded(action_result);

	std::cout <<"Optimization Local Planner Proccess ended !!!!!!!!!!!!!!!!!!!!!!!" << std::endl << "==================================================="<< std::endl << std::endl;
	resetFlags();

	for (size_t j = 0; j < vec_len_cat_opt.size(); j++){
		printf("Optimized length_catenary=[%f] for state=[%lu]\n",vec_len_cat_opt[j],j);
	}

  std::cout << "==================================================="<< std::endl << std::endl;
}


void OptimizerLocalPlanner::getPointsFromGlobalPath(trajectory_msgs::MultiDOFJointTrajectory _path, vector<Eigen::Vector3d> &v_ugv_, vector<Eigen::Vector3d> &v_uav_)
{
	float x_ugv_, y_ugv_, z_ugv_;
	float x_uav_, y_uav_, z_uav_;
	Eigen::Vector3d p_uav_ , p_ugv_;
	geometry_msgs::Quaternion qt_;

    double D_ugv_ = 0.0;
    double D_uav_ = 0.0;
	int n_ugv_ = 0;
	int n_uav_ = 0;
	v_ugv_.clear();
	v_uav_.clear();
	int count_ = 1; 
	
    for (size_t i = 0; i < _path.points.size()-1; i++)
    {
		// Get position and rotation vector for UGV
		x_ugv_ = _path.points.at(i+1).transforms[0].translation.x - _path.points.at(i).transforms[0].translation.x;
		y_ugv_ = _path.points.at(i+1).transforms[0].translation.y - _path.points.at(i).transforms[0].translation.y;
		z_ugv_ = _path.points.at(i+1).transforms[0].translation.z - _path.points.at(i).transforms[0].translation.z;
        // D_ugv_ = sqrt(x_ugv_ * x_ugv_ + y_ugv_ * y_ugv_ + z_ugv_ * z_ugv_);
		// Get position and rotation vector for UAV
		x_uav_ = _path.points.at(i+1).transforms[1].translation.x - _path.points.at(i).transforms[1].translation.x;
		y_uav_ = _path.points.at(i+1).transforms[1].translation.y - _path.points.at(i).transforms[1].translation.y;
		z_uav_ = _path.points.at(i+1).transforms[1].translation.z - _path.points.at(i).transforms[1].translation.z;
        D_uav_ = sqrt(x_uav_ * x_uav_ + y_uav_ * y_uav_ + z_uav_ * z_uav_);
		//First analize if distance between points in bigger that the maximum distance to create a new point
		if (D_uav_ > min_distance_add_new_point){
			// Get position and rotation vector for UGV
			n_uav_ = floor(D_uav_ / min_distance_add_new_point);
			double xp_ugv_ = x_ugv_/((double)n_uav_+1.0);
			double yp_ugv_ = y_ugv_/((double)n_uav_+1.0);
			double zp_ugv_ = z_ugv_/((double)n_uav_+1.0);
			p_ugv_.x() = _path.points.at(i).transforms[0].translation.x;
			p_ugv_.y() = _path.points.at(i).transforms[0].translation.y;
			p_ugv_.z() = _path.points.at(i).transforms[0].translation.z;
			v_ugv_.push_back(p_ugv_);
			qt_.x = _path.points.at(i).transforms[0].rotation.x;
			qt_.y = _path.points.at(i).transforms[0].rotation.y;
			qt_.z = _path.points.at(i).transforms[0].rotation.z;
			qt_.w = _path.points.at(i).transforms[0].rotation.w;
			vec_rot_ugv.push_back(qt_);
			// printf("point =[%i] getPointsFromGlobalPathUGV = [%f %f %f] \n",count_,p_ugv_.x(),p_ugv_.y(),p_ugv_.z());
			// Get position and rotation vector for UAV
			double xp_uav_ = x_uav_/((double)n_uav_+1.0);
			double yp_uav_ = y_uav_/((double)n_uav_+1.0);
			double zp_uav_ = z_uav_/((double)n_uav_+1.0);
			p_uav_.x() = _path.points.at(i).transforms[1].translation.x;
			p_uav_.y() = _path.points.at(i).transforms[1].translation.y;
			p_uav_.z() = _path.points.at(i).transforms[1].translation.z;
			v_uav_.push_back(p_uav_);
			qt_.x = _path.points.at(i).transforms[1].rotation.x;
			qt_.y = _path.points.at(i).transforms[1].rotation.y;
			qt_.z = _path.points.at(i).transforms[1].rotation.z;
			qt_.w = _path.points.at(i).transforms[1].rotation.w;
			vec_rot_uav.push_back(qt_);
			// printf("point =[%i] getPointsFromGlobalPathUAV = [%f %f %f]\n",count_,p_uav_.x(),p_uav_.y(),p_uav_.z());
			count_++;
		}
		else{
			// Get position and rotation vector for UGV
			p_ugv_.x() = _path.points.at(i).transforms[0].translation.x;
			p_ugv_.y() = _path.points.at(i).transforms[0].translation.y;
			p_ugv_.z() = _path.points.at(i).transforms[0].translation.z;
			v_ugv_.push_back(p_ugv_);
			qt_.x = _path.points.at(i).transforms[0].rotation.x;
			qt_.y = _path.points.at(i).transforms[0].rotation.y;
			qt_.z = _path.points.at(i).transforms[0].rotation.z;
			qt_.w = _path.points.at(i).transforms[0].rotation.w;
			vec_rot_ugv.push_back(qt_);
			// printf("point =[%i] getPointsFromGlobalPathUGV = [%f %f %f]\n",count_,p_ugv_.x(),p_ugv_.y(),p_ugv_.z());
			// Get position and rotation vector for UAV
			p_uav_.x() = _path.points.at(i).transforms[1].translation.x;
			p_uav_.y() = _path.points.at(i).transforms[1].translation.y;
			p_uav_.z() = _path.points.at(i).transforms[1].translation.z;
			v_uav_.push_back(p_uav_);
			qt_.x = _path.points.at(i).transforms[1].rotation.x;
			qt_.y = _path.points.at(i).transforms[1].rotation.y;
			qt_.z = _path.points.at(i).transforms[1].rotation.z;
			qt_.w = _path.points.at(i).transforms[1].rotation.w;
			vec_rot_uav.push_back(qt_);
			// printf("point =[%i] getPointsFromGlobalPathUAV = [%f %f %f]\n",count_,p_uav_.x(),p_uav_.y(),p_uav_.z());
			count_++;
		}
    }
	// Get position and rotation vector for UGV
	p_ugv_.x() = _path.points.at(_path.points.size()-1).transforms[0].translation.x;
	p_ugv_.y() = _path.points.at(_path.points.size()-1).transforms[0].translation.y;
	p_ugv_.z() = _path.points.at(_path.points.size()-1).transforms[0].translation.z;
	v_ugv_.push_back(p_ugv_);
	qt_.x = _path.points.at(_path.points.size()-1).transforms[0].rotation.x;
	qt_.y = _path.points.at(_path.points.size()-1).transforms[0].rotation.y;
	qt_.z = _path.points.at(_path.points.size()-1).transforms[0].rotation.z;
	qt_.w = _path.points.at(_path.points.size()-1).transforms[0].rotation.w;
	vec_rot_ugv.push_back(qt_);
	// printf("point =[%i] getPointsFromGlobalPathUGV = [%f %f %f]\n",count_,p_ugv_.x(),p_ugv_.y(),p_ugv_.z());
	// Get position and rotation vector for UAV
	p_uav_.x() = _path.points.at(_path.points.size()-1).transforms[1].translation.x;
	p_uav_.y() = _path.points.at(_path.points.size()-1).transforms[1].translation.y;
	p_uav_.z() = _path.points.at(_path.points.size()-1).transforms[1].translation.z;
	v_uav_.push_back(p_uav_);
	qt_.x = _path.points.at(_path.points.size()-1).transforms[1].rotation.x;
	qt_.y = _path.points.at(_path.points.size()-1).transforms[1].rotation.y;
	qt_.z = _path.points.at(_path.points.size()-1).transforms[1].rotation.z;
	qt_.w = _path.points.at(_path.points.size()-1).transforms[1].rotation.w;
	vec_rot_uav.push_back(qt_);
	// printf("point =[%i] getPointsFromGlobalPathUAV = [%f %f %f]\n",count_,p_uav_.x(),p_uav_.y(),p_uav_.z());
	count_++;

	for (size_t i = 0 ; i <  v_ugv_.size() ; i++){
		vec_pose_init_ugv.push_back(v_ugv_[i]);
	}
	for (size_t i = 0 ; i <  v_uav_.size() ; i++){
		vec_pose_init_uav.push_back(v_uav_[i]);
	}
}

void OptimizerLocalPlanner::calculateDistanceVertices(vector<double> &_v_D_ugv,vector<double> &_v_D_uav)
{
	float x, y, z;
	_v_D_ugv.clear();
	_v_D_uav.clear();
    double pL = 0.0;
	double sum_distance_ = 0.0;
    for (size_t i = 0; i < new_path_ugv.size()-1; i++)
    {
    	x = new_path_ugv[i+1].x() - new_path_ugv[i].x();
		y = new_path_ugv[i+1].y() - new_path_ugv[i].y();
		z = new_path_ugv[i+1].z() - new_path_ugv[i].z();
        pL= sqrt(x * x + y * y + z * z);
		sum_distance_ = sum_distance_ + pL;
		_v_D_ugv.push_back(pL);
    }
	initial_distance_states_ugv = sum_distance_ / (new_path_ugv.size()-1);

	pL = 0.0;
	sum_distance_ = 0.0;
	for (size_t i = 0; i < new_path_uav.size()-1; i++)
    {
    	x = new_path_uav[i+1].x() - new_path_uav[i].x();
		y = new_path_uav[i+1].y() - new_path_uav[i].y();
		z = new_path_uav[i+1].z() - new_path_uav[i].z();
        pL= sqrt(x * x + y * y + z * z);
		sum_distance_ = sum_distance_ + pL;
		_v_D_uav.push_back(pL);
    }
	initial_distance_states_uav = sum_distance_ / (new_path_uav.size()-1);
}

void OptimizerLocalPlanner::getTemporalState(vector<double> &_time_ugv, vector<double> &_time_uav) 
{
	_time_ugv.clear();
	_time_uav.clear();
	double _dT = 0;
	for (size_t i= 0 ; i < new_path_ugv.size(); i++){
		if (i == 0){
			_time_ugv.push_back(0.0);
		}
		else{
			_dT = (vec_dist_init_ugv[i-1])/initial_velocity_ugv;
			_time_ugv.push_back(_dT);
		}
	}
	_dT = 0;
	for (size_t i= 0 ; i < new_path_uav.size(); i++){
		if (i == 0){
			_time_uav.push_back(0.0);
		}
		else{
			_dT = (vec_dist_init_uav[i-1])/initial_velocity_uav;
			_time_uav.push_back(_dT );
		}
	}
}

geometry_msgs::Point OptimizerLocalPlanner::getReelPoint(const float px_, const float py_, const float pz_,const float qx_, const float qy_, const float qz_, const float qw_, geometry_msgs::Point p_reel_)
{
	geometry_msgs::Point ret;

	double roll_, pitch_, yaw_;
	tf::Quaternion q_(qx_,qy_,qz_,qw_);
	tf::Matrix3x3 M_(q_);	
	M_.getRPY(roll_, pitch_, yaw_);

	double lengt_vec =  sqrt(p_reel_.x*p_reel_.x + p_reel_.y*p_reel_.y);
	ret.x = px_ + lengt_vec *cos(yaw_); 
	ret.y = py_ + lengt_vec *sin(yaw_);
	ret.z = pz_ + p_reel_.z ;

	// printf("pos_reel = [%f %f %f] yaw = %f\n",pos_reel.x,pos_reel.y,pos_reel.z,yaw_ugv);

	return ret;
}

