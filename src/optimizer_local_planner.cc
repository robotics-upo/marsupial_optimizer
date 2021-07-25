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

	nh->param<double>("w_alpha_ugv", w_alpha_ugv,0.1);
	nh->param<double>("w_alpha_uav", w_alpha_uav,0.1);
	nh->param<double>("w_beta_uav",	w_beta_uav,0.1);
	nh->param<double>("w_iota_ugv",	w_iota_ugv,0.1);
	nh->param<double>("w_iota_uav",	w_iota_uav,0.1);
	nh->param<double>("w_beta_ugv",	w_beta_ugv,0.1);
	nh->param<double>("w_theta_ugv",	w_theta_ugv,0.1);
	nh->param<double>("w_gamma_uav", w_gamma_uav,0.1);
	nh->param<double>("w_gamma_ugv", w_gamma_ugv,0.1);
	nh->param<double>("w_kappa_ugv", w_kappa_ugv,0.1);
	nh->param<double>("w_kappa_uav", w_kappa_uav,0.1);
	nh->param<double>("w_delta_ugv", w_delta_ugv,0.1);
	nh->param<double>("w_delta", w_delta,0.1);
	nh->param<double>("w_epsilon_ugv", w_epsilon_ugv,0.1);
	nh->param<double>("w_epsilon", w_epsilon,0.1);
	nh->param<double>("w_zeta_uav", w_zeta_uav,0.1);
	nh->param<double>("w_zeta_ugv", w_zeta_ugv,0.1);
	nh->param<double>("w_mu_uav", w_mu_uav,0.1);
	nh->param<double>("w_nu_ugv", w_nu_ugv,0.1);
	
	nh->param<double>("w_eta_1", w_eta_1,0.1);
	nh->param<double>("w_eta_2", w_eta_2,0.1);
	nh->param<double>("w_eta_3", w_eta_3,0.1);
	nh->param<double>("w_lambda", w_lambda,0.1);

	nh->param<int>("n_iter_opt", n_iter_opt,200);
	nh->param<double>("distance_obstacle_ugv", distance_obstacle_ugv,0.5);
	nh->param<double>("distance_obstacle_uav", distance_obstacle_uav,1.0);
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

	nh->param<double>("length_tether_max", length_tether_max,20.0);

	nh->param<bool>("verbose_optimizer",verbose_optimizer, true);

	nh->param<bool>("write_data_for_analysis",write_data_for_analysis, false);
	nh->param("path", path, (std::string) "~/");
	nh->param("name_output_file", name_output_file, (std::string) "optimization_test");
	nh->param<int>("scenario_number", scenario_number, 1);
	nh->param<int>("num_pos_initial", num_pos_initial, 1);
	nh->param<int>("num_goal", num_goal, 0);
    nh->param("use_octomap_local", use_octomap_local, (bool)false);

	nh->param("debug", debug, (bool)0);
 	nh->param("show_config", showConfig, (bool)0);

	ROS_INFO_COND(showConfig, PRINTF_BLUE "Optimizer Local Planner 3D Node Configuration:\n");
	// ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Trajectory Position Increments = [%.2f], Tolerance: [%.2f]", traj_dxy_max, traj_pos_tol);
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
	mp_.clearMarkersPointLines(points_ugv_marker, lines_ugv_marker,traj_marker_ugv_pub_,20);
  	mp_.clearMarkersPointLines(points_uav_marker, lines_uav_marker,traj_marker_uav_pub_,20);
	ROS_INFO(PRINTF_BLUE"alpha_uav=[%f] alpha_ugv=[%f] beta_uav=[%f] beta_ugv=[%f] iota_uav=[%f] theta_ugv=[%f] gamma_uav=[%f] gamma_ugv=[%f] kappa_ugv=[%f] kappa_uav=[%f] delta=[%f] epsilon=[%f] zeta=[%f] eta_1=[%f] lambda=[%f]",
						 w_alpha_uav, w_alpha_ugv, w_beta_uav, w_beta_ugv, w_iota_uav, w_theta_ugv,w_gamma_uav, w_gamma_ugv, w_kappa_ugv, 
						 w_kappa_uav, w_delta,w_epsilon,w_zeta_uav, w_eta_1, w_lambda);
	ROS_INFO(PRINTF_BLUE"Optimizer_Local_Planner: Parameters loaded, ready for Optimization Process. Waiting for Action!!");
	configServices();
}

void OptimizerLocalPlanner::initializeSubscribers()
{
    if (use_octomap_local)
    {
        local_map_sub = nh->subscribe<octomap_msgs::Octomap>("/octomap_binary_local", 1, &OptimizerLocalPlanner::collisionMapCallBack, this);
    }
    else
    {
        // local_map_sub = nh->subscribe<PointCloud>("/points", 1, &OptimizerLocalPlanner::pointsSub, this);
    }
    
    local_trav_map_sub = nh->subscribe<octomap_msgs::Octomap>("/oct_trav/octomap_binary", 1, &OptimizerLocalPlanner::traversableMapCallBack, this);
	octomap_ws_sub_ = nh->subscribe( "/octomap_point_cloud_centers", 1,  &OptimizerLocalPlanner::readOctomapCallback, this);
    point_cloud_ugv_traversability_sub_ = nh->subscribe( "/region_growing_traversability_pc_map", 1,  &OptimizerLocalPlanner::readPointCloudTraversabilityUGVCallback, this);
    point_cloud_ugv_obstacles_sub_ = nh->subscribe( "/region_growing_obstacles_pc_map", 1,  &OptimizerLocalPlanner::readPointCloudObstaclesUGVCallback, this);

    clean_markers_sub_ = nh->subscribe( "/clean_marker_optimizer", 1,  &OptimizerLocalPlanner::deleteMarkersCallBack, this);

    trajPub = nh->advertise<trajectory_msgs::MultiDOFJointTrajectory>("local_path", 1);

    
	ROS_INFO(PRINTF_BLUE"Optimizer_Local_Planner: Subscribers Initialized");
}

void OptimizerLocalPlanner::initializePublishers()
{
  	post_traj_marker_ugv_pub_ = nh->advertise<visualization_msgs::MarkerArray>("post_init_trajectory_ugv_marker", 2);
  	traj_marker_ugv_pub_ = nh->advertise<visualization_msgs::MarkerArray>("opt_trajectory_ugv_marker", 2);
  	traj_marker_uav_pub_ = nh->advertise<visualization_msgs::MarkerArray>("opt_trajectory_uav_marker", 2);
	catenary_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("catenary_marker", 100);

  	ROS_INFO(PRINTF_BLUE"Optimizer_Local_Planner: Publishers Initialized");
}

void OptimizerLocalPlanner::resetFlags()
{
  	mapReceivedFull = false;
  	mapReceivedTrav = false;
}

void OptimizerLocalPlanner::cleanVectors()
{
	new_path_ugv.clear();
	new_path_uav.clear();	
	vec_init_rot_ugv.clear(); 
	vec_init_rot_uav.clear();
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
	ROS_INFO(PRINTF_MAGENTA "KDTree : Preparing KDTree UAV");
	nn_uav.setInput(*msg);
  	ROS_INFO_COND(debug, PRINTF_BLUE "Local Planner: Received Point Cloud for KDTree UAV");

}

void OptimizerLocalPlanner::readPointCloudTraversabilityUGVCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	ROS_INFO(PRINTF_MAGENTA "KDTree : Preparing KDTree UGV Traversability");
	nn_trav.setInput(*msg);
  	ROS_INFO_COND(debug, PRINTF_BLUE "Local Planner: Received Point Cloud for KDTree traversability UGV");
}

void OptimizerLocalPlanner::readPointCloudObstaclesUGVCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	ROS_INFO(PRINTF_MAGENTA "KDTree : Preparing KDTree UGV Obstacles");
	nn_ugv_obs.setInput(*msg);
  	ROS_INFO_COND(debug, PRINTF_BLUE "Local Planner: Received Point Cloud for KDTree obstacles UGV");
}

void OptimizerLocalPlanner::collisionMapCallBack(const octomap_msgs::OctomapConstPtr &msg)
{
    mapReceivedFull = true;
	// map = msg;
	mapFull_msg = (octomap::OcTree *)octomap_msgs::binaryMsgToMap(*msg);
}

void OptimizerLocalPlanner::traversableMapCallBack(const octomap_msgs::OctomapConstPtr &msg)
{
    mapReceivedTrav = true;
	// map = msg;
	mapTrav_msg = (octomap::OcTree *)octomap_msgs::binaryMsgToMap(*msg);
}

void OptimizerLocalPlanner::deleteMarkersCallBack(const std_msgs::BoolConstPtr &msg)
{
	if (msg->data == true){
		mp_.clearMarkers(catenary_marker, 150, catenary_marker_pub_);
		mp_.clearMarkersPointLines(points_ugv_marker, lines_ugv_marker,traj_marker_ugv_pub_,30);
  		mp_.clearMarkersPointLines(points_uav_marker, lines_uav_marker,traj_marker_uav_pub_,30);
  		mp_.clearMarkersPointLines(post_points_ugv_marker, post_lines_ugv_marker, post_traj_marker_ugv_pub_,30);
	}
}

void OptimizerLocalPlanner::executeOptimizerPathPreemptCB()
{
    ROS_INFO_COND(debug, "Goal Preempted");
    execute_path_srv_ptr->setPreempted(); // set the action state to preempted

    resetFlags();
    mp_.clearMarkersPointLines(points_ugv_marker, lines_ugv_marker, traj_marker_ugv_pub_, 0);
    mp_.clearMarkersPointLines(points_uav_marker, lines_uav_marker, traj_marker_uav_pub_, 0);
	mp_.clearMarkersPointLines(post_points_ugv_marker, post_lines_ugv_marker, post_traj_marker_ugv_pub_,0);

}

void OptimizerLocalPlanner::executeOptimizerPathGoalCB()
{
  	ROS_INFO_COND(debug, PRINTF_GREEN "Optimizer Local Planner Goal received in action server mode");
	// ros::Duration(0.5).sleep();
	auto path_shared_ptr = execute_path_srv_ptr->acceptNewGoal();
  	globalTrajectory = path_shared_ptr->path;
	vec_len_cat_init = path_shared_ptr->length_catenary; 
	// printf("vec_len_cat_init.size=[%lu]\n",vec_len_cat_init.size());
	printf("\tReceiving Nodes from Path Planner to carry out pre-processing for Optimization (pos_ugv/pos_uav/length_cat) : \n");
	for (size_t i = 0; i < globalTrajectory.points.size(); i++){
		printf("point=[%lu/%lu] GlobalTrayectoryPointTranslation=[%f %f %f / %f %f %f / %f]\n",i+1, globalTrajectory.points.size(),
		globalTrajectory.points.at(i).transforms[0].translation.x, globalTrajectory.points.at(i).transforms[0].translation.y, globalTrajectory.points.at(i).transforms[0].translation.z,
		globalTrajectory.points.at(i).transforms[1].translation.x, globalTrajectory.points.at(i).transforms[1].translation.y, globalTrajectory.points.at(i).transforms[1].translation.z,
		vec_len_cat_init[i]);
	}

	getPointsFromGlobalPath(globalTrajectory, new_path_ugv, new_path_uav);

    auto size = new_path_uav.size();
    // ROS_INFO_COND(debug, PRINTF_GREEN "Number of Vertices to optimize = [%lu] = size",size);

	mp_.clearMarkers(catenary_marker, 150, catenary_marker_pub_);

  	mp_.clearMarkersPointLines(points_ugv_marker, lines_ugv_marker,traj_marker_ugv_pub_,size);
  	mp_.clearMarkersPointLines(points_uav_marker, lines_uav_marker,traj_marker_uav_pub_,size);
	mp_.clearMarkersPointLines(post_points_ugv_marker, post_lines_ugv_marker, post_traj_marker_ugv_pub_,size);

	calculateDistanceVertices(vec_dist_init_ugv, vec_dist_init_uav);
 	getTemporalState(vec_time_init_ugv, vec_time_init_uav);

	dm_.initDataManagement(path, name_output_file, scenario_number, num_pos_initial, num_goal, initial_velocity_ugv, initial_velocity_uav, 
							initial_acceleration_ugv, initial_acceleration_uav, pos_reel_ugv, vec_pose_init_ugv, vec_pose_init_uav, 
							vec_len_cat_init, vec_init_rot_ugv, vec_init_rot_uav, mapFull_msg, mapTrav_msg);			   
	getKinematicTrajectory(vec_pose_init_ugv, vec_pose_init_uav, v_init_angles_kinematic_ugv, v_init_angles_kinematic_uav);
	dm_.writeTemporalDataBeforeOptimization(vec_dist_init_ugv, vec_dist_init_uav, vec_time_init_ugv, vec_time_init_uav,
											v_init_angles_kinematic_ugv, v_init_angles_kinematic_uav);

	std::cout << std::endl <<  "==================================================="  << std::endl << "\tPreparing to execute Optimization: Creating Parameter Blocks!!" << std::endl;

  	Problem problem;

	//Set Parameter Blocks from path global information
	statesPosUGV.clear();
	statesPosUAV.clear();
	statesRotUGV.clear();
	statesRotUAV.clear();
	statesTimeUGV.clear();
	statesTimeUAV.clear();
	statesLength.clear();
	printf("[_] states.parameter = [  pos_x_\tpos_y_\tpos_z_\trot_x_\trot_y_\trot_z_\trot_w_\ttime_\tlength_]\n");
	for (size_t i = 0 ; i < new_path_uav.size(); i ++){
		parameterBlockPos parameter_block_pos_ugv;
		parameterBlockPos parameter_block_pos_uav;
		parameterBlockRot parameter_block_rot_ugv;
		parameterBlockRot parameter_block_rot_uav;
		parameterBlockTime parameter_block_time_ugv;
		parameterBlockTime parameter_block_time_uav;
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
		parameter_block_rot_ugv.parameter[0] = i; //id parameter
		parameter_block_rot_ugv.parameter[1] = vec_init_rot_ugv[i].x;
		parameter_block_rot_ugv.parameter[2] = vec_init_rot_ugv[i].y; 
		parameter_block_rot_ugv.parameter[3] = vec_init_rot_ugv[i].z; 
		parameter_block_rot_ugv.parameter[4] = vec_init_rot_ugv[i].w;
		parameter_block_rot_uav.parameter[0] = i; //id parameter
		parameter_block_rot_uav.parameter[1] = vec_init_rot_uav[i].x;
		parameter_block_rot_uav.parameter[2] = vec_init_rot_uav[i].y; 
		parameter_block_rot_uav.parameter[3] = vec_init_rot_uav[i].z; 
		parameter_block_rot_uav.parameter[4] = vec_init_rot_uav[i].w; 
		//Time Parameters for UGV and UAV
		parameter_block_time_ugv.parameter[0] = i; //id parameter
		parameter_block_time_ugv.parameter[1] = vec_time_init_ugv[i];
		parameter_block_time_uav.parameter[0] = i; //id parameter
		parameter_block_time_uav.parameter[1] = vec_time_init_uav[i];
		//Length Cable Parameters for UGV and UAV
		parameter_block_length.parameter[0] = i; //id parameter
		parameter_block_length.parameter[1] = vec_len_cat_init[i];

		statesPosUGV.push_back(parameter_block_pos_ugv);
		statesPosUAV.push_back(parameter_block_pos_uav);
		statesRotUGV.push_back(parameter_block_rot_ugv);
		statesRotUAV.push_back(parameter_block_rot_uav);
		statesTimeUGV.push_back(parameter_block_time_ugv);
		statesTimeUAV.push_back(parameter_block_time_uav);
		statesLength.push_back(parameter_block_length);

		//Convert quaternion to euler angles
		double roll_ugv_init_, pitch_ugv_init_, yaw_ugv_init_;
		tf::Quaternion q_ugv_init_(parameter_block_rot_ugv.parameter[1],parameter_block_rot_ugv.parameter[2],parameter_block_rot_ugv.parameter[3],parameter_block_rot_ugv.parameter[4]);
		tf::Matrix3x3 M_ugv_init_(q_ugv_init_);	
		M_ugv_init_.getRPY(roll_ugv_init_, pitch_ugv_init_, yaw_ugv_init_);

		printf("[%lu]states.parameter: UGV=[%f %f %f / %f %f %f]",i+1,parameter_block_pos_ugv.parameter[1],parameter_block_pos_ugv.parameter[2],parameter_block_pos_ugv.parameter[3],
															roll_ugv_init_, pitch_ugv_init_, yaw_ugv_init_);
		printf(" UAV=[%f %f %f / %f %f %f %f] / [%f %f] / %f]\n", parameter_block_pos_uav.parameter[1],parameter_block_pos_uav.parameter[2],parameter_block_pos_uav.parameter[3],
															parameter_block_rot_uav.parameter[1],parameter_block_rot_uav.parameter[2],parameter_block_rot_uav.parameter[3],parameter_block_rot_uav.parameter[4],
															parameter_block_time_ugv.parameter[1],parameter_block_time_uav.parameter[1],parameter_block_length.parameter[1]);
	}
    
	/*** Cost Function I-i : Equidistance constrain UGV ***/
	printf("initial_distance_states_ugv = %f count_fix_points_ugv = %i \n",initial_distance_states_ugv, count_fix_points_ugv);
	for (int i = 0; i <  statesPosUGV.size() - 1 ; ++i) {
		CostFunction* cost_function1_1  = new AutoDiffCostFunction<EquiDistanceFunctorUGV, 1, 4, 4>
										 (new EquiDistanceFunctorUGV(w_alpha_ugv, 
										 							 initial_distance_states_ugv)); 
		problem.AddResidualBlock(cost_function1_1, NULL, statesPosUGV[i].parameter, 
														 statesPosUGV[i+1].parameter);
		// if (i == 0)
		// 	problem.SetParameterBlockConstant(statesPosUGV[i].parameter);
		if (i < count_fix_points_ugv)
			problem.SetParameterBlockConstant(statesPosUGV[i].parameter);
		// if (i == (statesPosUGV.size() - 2))
		// 	problem.SetParameterBlockConstant(statesPosUGV[i+1].parameter);
		problem.SetParameterLowerBound(statesPosUGV[i].parameter, 3, 0.0);
	}

	/*** Cost Function II-i : Length Trajectory constrain UGV***/
	for (int i = 0; i < statesPosUGV.size() - 6; ++i) {
    	CostFunction* cost_function2_0  = new AutoDiffCostFunction<LengthFunctorUGV, 1, 4, 4, 4, 4, 4, 4>
										 (new LengthFunctorUGV(w_nu_ugv, 
															   vec_dist_init_ugv[i],
															   vec_dist_init_ugv[i+1],
															   vec_dist_init_ugv[i+2],
															   vec_dist_init_ugv[i+3], 
															   vec_dist_init_ugv[i+4])); 
    	problem.AddResidualBlock(cost_function2_0, NULL, statesPosUGV[i].parameter, 
														 statesPosUGV[i+1].parameter,
														 statesPosUGV[i+2].parameter, 
														 statesPosUGV[i+3].parameter,
														 statesPosUGV[i+4].parameter,
														 statesPosUGV[i+5].parameter);
  	}

	/*** Cost Function II-i : Obstacles constrain UGV***/
	for (int i = 0; i < statesPosUGV.size(); ++i) {
    	CostFunction* cost_function2_1  = new AutoDiffCostFunction<ObstacleDistanceFunctorUGV::ObstaclesFunctorUGV, 1, 4>
										 (new ObstacleDistanceFunctorUGV::ObstaclesFunctorUGV(w_beta_ugv, distance_obstacle_ugv, 
										  nn_ugv_obs.kdtree, nn_ugv_obs.obs_points)); 
    	problem.AddResidualBlock(cost_function2_1, NULL, statesPosUGV[i].parameter);
  	}
	
	/*** Cost Function II-iii : Traversability constrain UGV***/
	for (int i = 0; i < statesPosUGV.size(); ++i) {
    	CostFunction* cost_function2_3  = new AutoDiffCostFunction<TraversabilityDistanceFunctorUGV::TraversabilityFunctor, 1, 4>
										 (new TraversabilityDistanceFunctorUGV::TraversabilityFunctor(w_theta_ugv, nn_trav.kdtree, 
										  nn_trav.obs_points)); 
    	problem.AddResidualBlock(cost_function2_3, NULL, statesPosUGV[i].parameter);
  	}

	/*** Cost Function III-i : Kinematic constrain UGV ***/
	for (int i = 0; i < statesPosUGV.size() - 2; ++i) {
		CostFunction* cost_function3_1  = new AutoDiffCostFunction<KinematicsFunctorUGV, 1, 4, 4, 4>
										 (new KinematicsFunctorUGV(w_gamma_ugv, angle_min_traj)); 
		problem.AddResidualBlock(cost_function3_1, NULL, statesPosUGV[i].parameter, statesPosUGV[i+1].parameter, statesPosUGV[i+2].parameter);
	}

	/*** Cost Function III-ii : Rotation constrain UGV ***/
	// for (int i = 0; i < statesRotUGV.size() - 1; ++i) {
	// 	double _roll, _pitch, _yaw;
	// 	tf::Quaternion _q(statesRotUGV[i+1].parameter[1],statesRotUGV[i+1].parameter[2],statesRotUGV[i+1].parameter[3],statesRotUGV[i+1].parameter[4]);
	// 	tf::Matrix3x3 _M(_q);	
	// 	_M.getRPY(_roll, _pitch, _yaw);
	// 	CostFunction* cost_function3_3  = new NumericDiffCostFunction<RotationFunctorUGV, CENTRAL, 1, 4, 4, 5>(new RotationFunctorUGV(w_kappa_ugv,_yaw)); 
	// 	problem.AddResidualBlock(cost_function3_3, NULL, statesPosUGV[i].parameter, statesPosUGV[i+1].parameter, statesRotUGV[i+1].parameter);
	// }

	/*** Cost Function IV-i : Time constrain UGV***/
	// printf("initial_time_ugv = %f \n", initial_time_ugv);
	// printf("min_T_ugv = %f \n", min_T_ugv);
	for (int i = 0; i < statesTimeUGV.size(); ++i) {
		CostFunction* cost_function4_1  = new AutoDiffCostFunction<TimeFunctorUGV, 1, 2>
										  (new TimeFunctorUGV(w_delta_ugv, initial_time_ugv)); 
		problem.AddResidualBlock(cost_function4_1, NULL, statesTimeUGV[i].parameter);
		if (i < count_fix_points_ugv) 
			problem.SetParameterBlockConstant(statesTimeUGV[i].parameter);
		else
			problem.SetParameterLowerBound(statesTimeUGV[i].parameter, 1, min_T_ugv);
	}

	/*** Cost Function V : Velocity constrain UGV***/
	for (int i = 0; i < statesTimeUGV.size() - 1; ++i) {
		CostFunction* cost_function5_1  = new AutoDiffCostFunction<VelocityFunctorUGV, 1, 4, 4, 2>
										 (new VelocityFunctorUGV(w_epsilon_ugv, initial_velocity_ugv, count_fix_points_ugv)); 
		problem.AddResidualBlock(cost_function5_1, NULL, statesPosUGV[i].parameter, statesPosUGV[i+1].parameter, statesTimeUGV[i+1].parameter);
	}

	/*** Cost Function VI-i : Acceleration constrain UGV***/
	for (int i = 0; i < statesTimeUGV.size() - 2; ++i) {
		CostFunction* cost_function6_1  = new AutoDiffCostFunction<AccelerationFunctorUGV, 1, 4, 4, 4, 2, 2>
										 (new AccelerationFunctorUGV(w_zeta_ugv, initial_acceleration_ugv, count_fix_points_ugv)); 
		problem.AddResidualBlock(cost_function6_1, NULL, statesPosUGV[i].parameter, statesPosUGV[i+1].parameter, statesPosUGV[i+2].parameter, 
														 statesTimeUGV[i+1].parameter, statesTimeUGV[i+2].parameter);
	}



	// /*** Cost Function II-ii : Obstacles Through constrain UGV***/
	// for (int i = 0; i < statesPosUGV.size()-1; ++i) {
    // 	CostFunction* cost_function2_2  = new AutoDiffCostFunction<ObstacleDistanceThroughFunctorUGV::ObstaclesThroughFunctorUGV, 1, 4, 4>
	// 										(new ObstacleDistanceThroughFunctorUGV::ObstaclesThroughFunctorUGV(w_iota_ugv, mapTrav_msg)); 
    // 	problem.AddResidualBlock(cost_function2_2, NULL, statesPosUGV[i].parameter, statesPosUGV[i+1].parameter);
  	// }
	/*** Cost Function II-iv : Obstacles Through constrain UAV***/
	// for (int i = 0; i < statesPosUAV.size()-1; ++i) {
    // 	CostFunction* cost_function2_5  = new AutoDiffCostFunction<ObstacleDistanceThroughFunctorUAV::ObstaclesThroughFunctor, 1, 4, 4>
	// 										(new ObstacleDistanceThroughFunctorUAV::ObstaclesThroughFunctor(w_iota_uav, mapFull_msg)); 
    // 	problem.AddResidualBlock(cost_function2_5, NULL, statesPosUAV[i].parameter, statesPosUAV[i+1].parameter);
  	// }



	/*** Cost Function I-ii : Equidistance constrain UAV ***/
	// printf("initial_distance_states_uav = %f \n",initial_distance_states_uav);
	for (int i = 0; i <  statesPosUAV.size() - 1 ; ++i) {
		CostFunction* cost_function1_2  = new AutoDiffCostFunction<EquiDistanceFunctorUAV, 1, 4, 4>
										 (new EquiDistanceFunctorUAV(w_alpha_uav, initial_distance_states_uav)); 
		problem.AddResidualBlock(cost_function1_2, NULL, statesPosUAV[i].parameter, statesPosUAV[i+1].parameter);
		if (i == 0)
			problem.SetParameterBlockConstant(statesPosUAV[i].parameter);
		if (i == (statesPosUAV.size() - 2))
			problem.SetParameterBlockConstant(statesPosUAV[i+1].parameter);
		problem.SetParameterLowerBound(statesPosUAV[i].parameter, 3, 0.0);
		problem.SetParameterUpperBound(statesPosUAV[i].parameter, 3, 20.0);
	}

	// printf("            distance_obstacle_uav = %f\n",distance_obstacle_uav);
	/*** Cost Function II-iii : Obstacles constrain UAV***/
	for (int i = 0; i < statesPosUAV.size(); ++i) {
    	CostFunction* cost_function2_4  = new AutoDiffCostFunction<ObstacleDistanceFunctorUAV::ObstaclesFunctor, 1, 4>
										 (new ObstacleDistanceFunctorUAV::ObstaclesFunctor(w_beta_uav, distance_obstacle_uav, nn_uav.kdtree, 
										  nn_uav.obs_points)); 
    	problem.AddResidualBlock(cost_function2_4, NULL, statesPosUAV[i].parameter);
  	}

	/*** Cost Function II-iii : Repulsion constrain UAV***/
	for (int i = 0; i < statesPosUAV.size(); ++i) {
    	CostFunction* cost_function2_6  = new AutoDiffCostFunction<RepulsionFunctorUAV::RepulsionFunctor, 1, 4>
										 (new RepulsionFunctorUAV::RepulsionFunctor(w_mu_uav, distance_obstacle_uav, 
																					   nn_uav.kdtree, nn_uav.obs_points, new_path_uav[i])); 
    	problem.AddResidualBlock(cost_function2_6, NULL, statesPosUAV[i].parameter);
  	}

	/*** Cost Function III-ii : Kinematic constrain UAV ***/
	for (int i = 0; i < statesPosUAV.size() - 2; ++i) {
		CostFunction* cost_function3_2  = new AutoDiffCostFunction<KinematicsFunctorUAV, 1, 4, 4, 4>
										 (new KinematicsFunctorUAV(w_gamma_uav, angle_min_traj)); 
		problem.AddResidualBlock(cost_function3_2, NULL, statesPosUAV[i].parameter, statesPosUAV[i+1].parameter, statesPosUAV[i+2].parameter);
	}

	/*** Cost Function III-iiV : Rotation constrain UAV ***/
	// for (int i = 0; i < statesRotUAV.size() - 1; ++i) {
	// 	CostFunction* cost_function3_4  = new NumericDiffCostFunction<RotationFunctorUGV, CENTRAL, 1, 5, 5>(new RotationFunctorUAV(w_kappa_uav)); 
	// 	problem.AddResidualBlock(cost_function3_4, NULL, statesRotUAV[i].parameter, statesRotUAV[i+1].parameter);
	// }

	/*** Cost Function IV-ii : Time constrain UAV***/
	for (int i = 0; i < statesTimeUAV.size(); ++i) {
		CostFunction* cost_function4_2  = new AutoDiffCostFunction<TimeFunctorUAV, 1, 2>
										 (new TimeFunctorUAV(w_delta, initial_velocity_uav)); 
		problem.AddResidualBlock(cost_function4_2, NULL, statesTimeUAV[i].parameter);
		if (i == 0)
			problem.SetParameterBlockConstant(statesTimeUAV[i].parameter);
		problem.SetParameterLowerBound(statesTimeUAV[i].parameter, 1, 0.0);
	}

	/*** Cost Function V : Velocity constrain UAV***/
	for (int i = 0; i < statesTimeUAV.size() - 1; ++i) {
		CostFunction* cost_function5_2  = new AutoDiffCostFunction<VelocityFunctorUAV, 1, 4, 4, 2>
										 (new VelocityFunctorUAV(w_epsilon, initial_velocity_uav)); 
		problem.AddResidualBlock(cost_function5_2, NULL, 
								statesPosUAV[i].parameter, statesPosUAV[i+1].parameter, 
								statesTimeUAV[i+1].parameter);
	}

	/*** Cost Function VI-ii : Acceleration constrain UAV***/
	for (int i = 0; i < statesTimeUAV.size() - 2; ++i) {
		CostFunction* cost_function6_2  = new AutoDiffCostFunction<AccelerationFunctorUAV, 1, 4, 4, 4, 2, 2>
										 (new AccelerationFunctorUAV(w_zeta_uav, initial_acceleration_uav)); 
		problem.AddResidualBlock(cost_function6_2, NULL, 
								statesPosUAV[i].parameter, statesPosUAV[i+1].parameter, statesPosUAV[i+2].parameter, 
								statesTimeUAV[i+1].parameter, statesTimeUAV[i+2].parameter);
	}


	/*** Cost Function VII : Catenary constrain  ***/
	// for (int i = 0; i < statesLength.size(); ++i) {
	// 	CostFunction* cost_function7  = new NumericDiffCostFunction<CatenaryFunctor, CENTRAL, 3, 4, 4, 5, 2>
	// 									(new CatenaryFunctor(w_eta*10, distance_catenary_obstacle, nn_uav.kdtree, nn_uav.obs_points, nn_trav.kdtree, nn_trav.obs_points, pos_reel_ugv, size, nh)); 
	// 	problem.AddResidualBlock(cost_function7, NULL, statesPosUAV[i].parameter, statesPosUGV[i].parameter, statesRotUGV[i].parameter, statesLength[i].parameter);
	// 	if (i == 0)
	// 		problem.SetParameterBlockConstant(statesLength[i].parameter);
	// 	else{
	// 		problem.SetParameterLowerBound(statesLength[i].parameter, 1, 0.1);
	// 		problem.SetParameterUpperBound(statesLength[i].parameter, 1, length_tether_max);
	// 	}	
	// }
	/*** Cost Function VII : Catenary constrain  ***/
	for (int i = 0; i < statesLength.size(); ++i) {
		CostFunction* cost_function7  = new NumericDiffCostFunction<CatenaryFunctor, CENTRAL, 3, 4, 4, 2>
									   (new CatenaryFunctor(w_eta_1, w_eta_2, w_eta_3, distance_catenary_obstacle, nn_uav.kdtree, 
									   	nn_uav.obs_points, nn_trav.kdtree, nn_trav.obs_points, pos_reel_ugv, size, pos_reel_z, 
										new_path_uav[i], nh)); 
		problem.AddResidualBlock(cost_function7, NULL, statesPosUAV[i].parameter, statesPosUGV[i].parameter, statesLength[i].parameter);
		if (i == 0)
			problem.SetParameterBlockConstant(statesLength[i].parameter);
		else{
			problem.SetParameterLowerBound(statesLength[i].parameter, 1, 0.1);
			problem.SetParameterUpperBound(statesLength[i].parameter, 1, length_tether_max);
		}
	}	

	/*** Cost Function VIII : Dynamic Catenary constrain  ***/
	// for (int i = 0; i < statesLength.size() - 1; ++i) {
	// 	CostFunction* cost_function8  = new AutoDiffCostFunction<DynamicCatenaryFunctor, 1, 2, 2>(new DynamicCatenaryFunctor(w_lambda, dynamic_catenary)); 
	// 	problem.AddResidualBlock(cost_function8, NULL, statesLength[i].parameter, statesLength[i+1].parameter);
	// }

	std::cout <<  "==================================================="  << std::endl << "Optimization  started !!" << std::endl;

	start_time_opt = ros::Time::now();
	ceres::Solve(options, &problem, &summary);
	std::cout << summary.FullReport() << "\n";
	final_time_opt = ros::Time::now();

	std::cout << std::endl <<"Optimization Proccess Completed !!!" << std::endl << "===================================================" << std::endl << "Saving Temporal data in txt file ..." << std::endl;

	vec_pose_ugv_opt.clear();
	vec_pose_uav_opt.clear();
	vec_time_ugv_opt.clear();
	vec_time_uav_opt.clear();
	vec_len_cat_opt.clear();
	vec_opt_rot_ugv.clear();
	vec_opt_rot_uav.clear();
	new_path_ugv.clear(); //clear the old path to set the optimizer solution as a new path
	new_path_uav.clear(); //clear the old path to set the optimizer solution as a new path

  	for (size_t i = 0; i < size; i++){
		Eigen::Vector3d position_ugv_ = Eigen::Vector3d(statesPosUGV[i].parameter[1],statesPosUGV[i].parameter[2],statesPosUGV[i].parameter[3]);
		vec_pose_ugv_opt.push_back(position_ugv_);
		Eigen::Vector3d position_uav_ = Eigen::Vector3d(statesPosUAV[i].parameter[1],statesPosUAV[i].parameter[2],statesPosUAV[i].parameter[3]);
		vec_pose_uav_opt.push_back(position_uav_);
		double vLC = statesLength[i].parameter[1];
		vec_len_cat_opt.push_back(vLC);
		vec_time_ugv_opt.push_back(statesTimeUGV[i].parameter[1]);
		vec_time_uav_opt.push_back(statesTimeUAV[i].parameter[1]);
		geometry_msgs::Quaternion rot_angle_ugv_, rot_angle_uav_;
		rot_angle_ugv_.x = statesRotUGV[i].parameter[1];
		rot_angle_ugv_.y = statesRotUGV[i].parameter[2];
		rot_angle_ugv_.z = statesRotUGV[i].parameter[3];
		rot_angle_ugv_.w = statesRotUGV[i].parameter[4];
		vec_opt_rot_ugv.push_back(rot_angle_ugv_);
		rot_angle_uav_.x = statesRotUAV[i].parameter[1];
		rot_angle_uav_.y = statesRotUAV[i].parameter[2];
		rot_angle_uav_.z = statesRotUAV[i].parameter[3];
		rot_angle_uav_.w = statesRotUAV[i].parameter[4];
		vec_opt_rot_uav.push_back(rot_angle_uav_);
	}

	mp_.clearMarkers(catenary_marker, 150, catenary_marker_pub_);  //To delete possibles outliers points 
	// Staff for fost-proccessing of catenary length
	postProcessingCatenary();
	
	for(size_t i = 0; i < statesPosUAV.size(); i++){
		std::vector<geometry_msgs::Point> points_catenary_final;
		points_catenary_final.clear();

		// geometry_msgs::Point p_reel_=getReelPoint(statesPosUGV[i].parameter[1],statesPosUGV[i].parameter[2],statesPosUGV[i].parameter[3],statesRotUGV[i].parameter[1],statesRotUGV[i].parameter[2],statesRotUGV[i].parameter[3], statesRotUGV[i].parameter[4],pos_reel_ugv);
		CatenarySolver cSX_;
		cSX_.setMaxNumIterations(100);
	  	// cSX_.solve(p_reel_.x, p_reel_.y, p_reel_.z, statesPosUAV[i].parameter[1], statesPosUAV[i].parameter[2], statesPosUAV[i].parameter[3], statesLength[i].parameter[1], points_catenary_final);
	  	// The Reel Position is consider above base_link_ugv
		cSX_.solve(statesPosUGV[i].parameter[1], statesPosUGV[i].parameter[2], statesPosUGV[i].parameter[3]+pos_reel_z,
		  		   statesPosUAV[i].parameter[1], statesPosUAV[i].parameter[2], statesPosUAV[i].parameter[3], 
				   statesLength[i].parameter[1], points_catenary_final);
		double _d_ = sqrt(pow(statesPosUGV[i].parameter[1] - statesPosUAV[i].parameter[1],2) + 
						  pow(statesPosUGV[i].parameter[2] - statesPosUAV[i].parameter[2],2) + 
						  pow((statesPosUGV[i].parameter[3]+pos_reel_z) - statesPosUAV[i].parameter[3],2));

		//Graph final catenary states
		mp_.markerPoints(catenary_marker, points_catenary_final, i, size, catenary_marker_pub_);
		// double _d_ = sqrt(pow(p_reel_.x -statesPosUAV[i].parameter[1],2) + pow(p_reel_.y - statesPosUAV[i].parameter[2],2) + pow(p_reel_.z-statesPosUAV[i].parameter[3],2));
		//Convert quaternion to euler angles
		double roll_ugv_, pitch_ugv_, yaw_ugv_;
		tf::Quaternion q_ugv_(statesRotUGV[i].parameter[1],statesRotUGV[i].parameter[2],statesRotUGV[i].parameter[3],statesRotUGV[i].parameter[4]);
		tf::Matrix3x3 M_(q_ugv_);	
		M_.getRPY(roll_ugv_, pitch_ugv_, yaw_ugv_);
		printf("[%lu]states.parameter: UGV=[%f %f %f / %f %f %f] UAV=[%f %f %f / %f %f %f %f] , t=[%f/%f], l=[%f/points=%lu] d=[%f]\n", i+1,
																							statesPosUGV[i].parameter[1], statesPosUGV[i].parameter[2], statesPosUGV[i].parameter[3],
																							roll_ugv_, pitch_ugv_, yaw_ugv_,
																							statesPosUAV[i].parameter[1], statesPosUAV[i].parameter[2], statesPosUAV[i].parameter[3],
																							statesRotUAV[i].parameter[1], statesRotUAV[i].parameter[2], statesRotUAV[i].parameter[3], statesRotUAV[i].parameter[4],
																							statesTimeUGV[i].parameter[1],statesTimeUAV[i].parameter[1],
																							statesLength[i].parameter[1],points_catenary_final.size(),_d_);
	}
	
	mp_.getMarkerPoints(points_ugv_marker, vec_pose_ugv_opt, "points_ugv_m",3);	// RED: colour_ = 0 ; GREEN : colour_ = 1 ; BLUE: colour_ = 2 ; YELLOW = 3 ; PURPLE = 4;
	mp_.getMarkerLines(lines_ugv_marker, vec_pose_ugv_opt, "lines_ugv_m",3);
	mp_.getMarkerPoints(points_uav_marker, vec_pose_uav_opt, "points_uav_m",1);
	mp_.getMarkerLines(lines_uav_marker, vec_pose_uav_opt, "lines_uav_m",1);
	traj_marker_ugv_pub_.publish(points_ugv_marker);
	traj_marker_ugv_pub_.publish(lines_ugv_marker);
	traj_marker_uav_pub_.publish(points_uav_marker);
	traj_marker_uav_pub_.publish(lines_uav_marker);

	getKinematicTrajectory(vec_pose_ugv_opt, vec_pose_uav_opt, v_opt_angles_kinematic_ugv, v_opt_angles_kinematic_uav);
	dm_.writeTemporalDataAfterOptimization(size, vec_pose_ugv_opt, vec_pose_uav_opt, vec_time_ugv_opt, vec_time_uav_opt, 
	vec_len_cat_opt, vec_opt_rot_ugv, vec_opt_rot_uav, v_opt_angles_kinematic_ugv, v_opt_angles_kinematic_uav);
	std::cout << "...Temporal data saved in txt file." << std::endl << std::endl;
	int n_coll_opt_traj_ugv_, n_coll_init_path_ugv_, n_coll_opt_traj_uav_, n_coll_init_path_uav_;
	n_coll_opt_traj_ugv_ = n_coll_init_path_ugv_ = n_coll_opt_traj_uav_ = n_coll_init_path_uav_ = 0;
	if (write_data_for_analysis){
		std::cout << "Saving data for analisys in txt file..." << std::endl;
		double opt_compute_time_ = (final_time_opt - start_time_opt).toSec(); //Compute Time Optimization
		dm_.getDataForOptimizerAnalysis(nn_ugv_obs.kdtree, nn_uav.kdtree, nn_ugv_obs.obs_points, nn_uav.obs_points, opt_compute_time_, "UGV", n_coll_init_path_ugv_, n_coll_opt_traj_ugv_);
		dm_.getDataForOptimizerAnalysis(nn_uav.kdtree, nn_uav.kdtree, nn_uav.obs_points, nn_uav.obs_points, opt_compute_time_, "UAV", n_coll_init_path_uav_, n_coll_opt_traj_uav_);
		std::cout << "... data for analysis saved in txt file." << std::endl << std::endl;
	}

	cleanVectors();		//Clear vector after optimization and previus the next iteration
	ros::Duration(10.0).sleep();

	if (n_coll_opt_traj_ugv_ == 0 && n_coll_opt_traj_uav_ == 0){
		ROS_INFO("		Optimizer Local Planner: Goal position successfully achieved through optimization trajectory\n");
		action_result.arrived = true;
		execute_path_srv_ptr->setSucceeded(action_result);
	}
	else{
		ROS_ERROR("		Optimizer Local Planner: Goal position Not achieved through optimization trajectory\n");
		action_result.arrived = false;
		execute_path_srv_ptr->setSucceeded(action_result);
	}

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
	// int n_ugv_ = 0;
	// int n_uav_ = 0;
	v_ugv_.clear();
	v_uav_.clear();
	int count_ = 1; 
	count_fix_points_ugv = 1;
	count_not_fix_points_ugv = 0;
	int aux_cont_ = 0;

    for (size_t i = 0; i < _path.points.size()-1; i++)
    {
		// Get position and rotation vector for UGV
		x_ugv_ = _path.points.at(i+1).transforms[0].translation.x - _path.points.at(i).transforms[0].translation.x;
		y_ugv_ = _path.points.at(i+1).transforms[0].translation.y - _path.points.at(i).transforms[0].translation.y;
		z_ugv_ = _path.points.at(i+1).transforms[0].translation.z - _path.points.at(i).transforms[0].translation.z;
		
		// Fallowing lines to count the number of UGV initial nodes in the same position 
        D_ugv_ = sqrt(x_ugv_ * x_ugv_ + y_ugv_ * y_ugv_ + z_ugv_ * z_ugv_);
		if (D_ugv_ < 0.001){
			count_not_fix_points_ugv = count_not_fix_points_ugv + 0;
			if(aux_cont_ == i){  // To count the consecutive points that keep stable in the start point
				count_fix_points_ugv = count_fix_points_ugv + 1;
				aux_cont_++;
			}
		}
		else
			count_not_fix_points_ugv = count_not_fix_points_ugv + 1;

		// Get position and rotation vector for UAV
		x_uav_ = _path.points.at(i+1).transforms[1].translation.x - _path.points.at(i).transforms[1].translation.x;
		y_uav_ = _path.points.at(i+1).transforms[1].translation.y - _path.points.at(i).transforms[1].translation.y;
		z_uav_ = _path.points.at(i+1).transforms[1].translation.z - _path.points.at(i).transforms[1].translation.z;
        D_uav_ = sqrt(x_uav_ * x_uav_ + y_uav_ * y_uav_ + z_uav_ * z_uav_);
		//First analize if distance between points in bigger that the maximum distance to create a new point
		if (D_uav_ > min_distance_add_new_point){
			// Get position and rotation vector for UGV
				// n_uav_ = floor(D_uav_ / min_distance_add_new_point);
				// double xp_ugv_ = x_ugv_/((double)n_uav_+1.0);
				// double yp_ugv_ = y_ugv_/((double)n_uav_+1.0);
				// double zp_ugv_ = z_ugv_/((double)n_uav_+1.0);
			p_ugv_.x() = _path.points.at(i).transforms[0].translation.x;
			p_ugv_.y() = _path.points.at(i).transforms[0].translation.y;
			p_ugv_.z() = _path.points.at(i).transforms[0].translation.z;
			v_ugv_.push_back(p_ugv_);
			qt_.x = _path.points.at(i).transforms[0].rotation.x;
			qt_.y = _path.points.at(i).transforms[0].rotation.y;
			qt_.z = _path.points.at(i).transforms[0].rotation.z;
			qt_.w = _path.points.at(i).transforms[0].rotation.w;
			vec_init_rot_ugv.push_back(qt_);
			// Get position and rotation vector for UAV
				// double xp_uav_ = x_uav_/((double)n_uav_+1.0);
				// double yp_uav_ = y_uav_/((double)n_uav_+1.0);
				// double zp_uav_ = z_uav_/((double)n_uav_+1.0);
			p_uav_.x() = _path.points.at(i).transforms[1].translation.x;
			p_uav_.y() = _path.points.at(i).transforms[1].translation.y;
			p_uav_.z() = _path.points.at(i).transforms[1].translation.z;
			v_uav_.push_back(p_uav_);
			qt_.x = _path.points.at(i).transforms[1].rotation.x;
			qt_.y = _path.points.at(i).transforms[1].rotation.y;
			qt_.z = _path.points.at(i).transforms[1].rotation.z;
			qt_.w = _path.points.at(i).transforms[1].rotation.w;
			vec_init_rot_uav.push_back(qt_);
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
			vec_init_rot_ugv.push_back(qt_);
			// Get position and rotation vector for UAV
			p_uav_.x() = _path.points.at(i).transforms[1].translation.x;
			p_uav_.y() = _path.points.at(i).transforms[1].translation.y;
			p_uav_.z() = _path.points.at(i).transforms[1].translation.z;
			v_uav_.push_back(p_uav_);
			qt_.x = _path.points.at(i).transforms[1].rotation.x;
			qt_.y = _path.points.at(i).transforms[1].rotation.y;
			qt_.z = _path.points.at(i).transforms[1].rotation.z;
			qt_.w = _path.points.at(i).transforms[1].rotation.w;
			vec_init_rot_uav.push_back(qt_);
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
	vec_init_rot_ugv.push_back(qt_);
	// Get position and rotation vector for UAV
	p_uav_.x() = _path.points.at(_path.points.size()-1).transforms[1].translation.x;
	p_uav_.y() = _path.points.at(_path.points.size()-1).transforms[1].translation.y;
	p_uav_.z() = _path.points.at(_path.points.size()-1).transforms[1].translation.z;
	v_uav_.push_back(p_uav_);
	qt_.x = _path.points.at(_path.points.size()-1).transforms[1].rotation.x;
	qt_.y = _path.points.at(_path.points.size()-1).transforms[1].rotation.y;
	qt_.z = _path.points.at(_path.points.size()-1).transforms[1].rotation.z;
	qt_.w = _path.points.at(_path.points.size()-1).transforms[1].rotation.w;
	vec_init_rot_uav.push_back(qt_);
	count_++;

	for (size_t i = 0 ; i <  v_ugv_.size() ; i++){
		vec_pose_init_ugv.push_back(v_ugv_[i]);
	}
	for (size_t i = 0 ; i <  v_uav_.size() ; i++){
		vec_pose_init_uav.push_back(v_uav_[i]);
	}

	// To move al the states that have the same position through interpolation
	// double d_all_nodes = sqrt(pow(v_ugv_[0].x()-v_ugv_[ v_ugv_.size()-1].x(),2) +
	// 					 	  pow(v_ugv_[0].y()-v_ugv_[ v_ugv_.size()-1].y(),2) +
	// 					 	  pow(v_ugv_[0].z()-v_ugv_[ v_ugv_.size()-1].z(),2));
	// if (d_all_nodes > 0.001)
	interpolateFixedPointsPath(v_ugv_);
}

void OptimizerLocalPlanner::interpolateFixedPointsPath(vector<Eigen::Vector3d> &v_ugv_)
{
	vector<Eigen::Vector3d> vec_pose_init_ugv_aux_;
	vec_pose_init_ugv_aux_.clear();
	v_ugv_.clear();

	Eigen::Vector3d pos_ugv_;
	int count_ = 1;
	int init_pos_ = 0;
	int final_pos_ = 0;

	for (size_t i = 0; i < vec_pose_init_ugv.size()-1; i++)
    {
		if(i>=count_fix_points_ugv){
			// Get position and rotation vector for UGV
			double x_ugv_ = vec_pose_init_ugv[i+1].x() - vec_pose_init_ugv[i].x();
			double y_ugv_ = vec_pose_init_ugv[i+1].y() - vec_pose_init_ugv[i].y();
			double z_ugv_ = vec_pose_init_ugv[i+1].z() - vec_pose_init_ugv[i].z();
			double D_ugv_ = sqrt(x_ugv_ * x_ugv_ + y_ugv_ * y_ugv_ + z_ugv_ * z_ugv_);
			//First analize if distance between points in bigger that the maximum distance to create a new point
			if (D_ugv_ < 0.001){
				if (count_ == 1)
					init_pos_ = i ;
				count_++;
			}
			else if (D_ugv_ > 0.001 && count_ > 1){ 	
				final_pos_ = i +1 ;
				double x_ = vec_pose_init_ugv[final_pos_].x() - vec_pose_init_ugv[init_pos_].x();
				double y_ = vec_pose_init_ugv[final_pos_].y() - vec_pose_init_ugv[init_pos_].y();
				double z_ = vec_pose_init_ugv[final_pos_].z() - vec_pose_init_ugv[init_pos_].z();
				for(int j = 0; j < count_; j++){
					pos_ugv_.x() = vec_pose_init_ugv[init_pos_].x() + (x_/count_)*j;
					pos_ugv_.y() = vec_pose_init_ugv[init_pos_].y() + (y_/count_)*j;
					pos_ugv_.z() = vec_pose_init_ugv[init_pos_].z() + (z_/count_)*j;
					vec_pose_init_ugv_aux_.push_back(pos_ugv_);
				}
				count_ = 1;
			}
			else{
				vec_pose_init_ugv_aux_.push_back(vec_pose_init_ugv[i]);
			}
		}
		else{
				vec_pose_init_ugv_aux_.push_back(vec_pose_init_ugv[i]);
		}

		if ((i== vec_pose_init_ugv.size()-2 )&& (count_ > 1)){
			double x_ugv_ = vec_pose_init_ugv[vec_pose_init_ugv.size()-1].x() - vec_pose_init_ugv[vec_pose_init_ugv.size()-(1+count_)].x();
			double y_ugv_ = vec_pose_init_ugv[vec_pose_init_ugv.size()-1].y() - vec_pose_init_ugv[vec_pose_init_ugv.size()-(1+count_)].y();
			double z_ugv_ = vec_pose_init_ugv[vec_pose_init_ugv.size()-1].z() - vec_pose_init_ugv[vec_pose_init_ugv.size()-(1+count_)].z();
			for(int j = 1; j < count_; j++){
				// pos_ugv_.x() = vec_pose_init_ugv[vec_pose_init_ugv.size()-1].x();
				// pos_ugv_.y() = vec_pose_init_ugv[vec_pose_init_ugv.size()-1].y();
				// pos_ugv_.z() = vec_pose_init_ugv[vec_pose_init_ugv.size()-1].z();
				pos_ugv_.x() = vec_pose_init_ugv[vec_pose_init_ugv.size()-(1+count_)].x() + (x_ugv_/count_)*j;
				pos_ugv_.y() = vec_pose_init_ugv[vec_pose_init_ugv.size()-(1+count_)].y() + (y_ugv_/count_)*j;
				pos_ugv_.z() = vec_pose_init_ugv[vec_pose_init_ugv.size()-(1+count_)].z() + (z_ugv_/count_)*j;
				vec_pose_init_ugv_aux_.push_back(pos_ugv_);
			}
		}
    }
	int num_pos_ = vec_pose_init_ugv.size()-1;
	vec_pose_init_ugv_aux_.push_back(vec_pose_init_ugv[num_pos_]);

	//Set new values in vec_pos_init_ugv
	vec_pose_init_ugv.clear();
	vec_pose_init_ugv = vec_pose_init_ugv_aux_;
	v_ugv_ = vec_pose_init_ugv_aux_;

	// for (size_t j= 0 ; j < vec_pose_init_ugv.size() ; j++)
		// printf ("[%lu] vec_pose_init_ugv[%f %f %f]\n",j+1, vec_pose_init_ugv[j].x(),vec_pose_init_ugv[j].y(),vec_pose_init_ugv[j].z());

    auto s_ = vec_pose_init_ugv.size();
	mp_.clearMarkersPointLines(post_points_ugv_marker, post_lines_ugv_marker, post_traj_marker_ugv_pub_,s_);
	mp_.getMarkerPoints(post_points_ugv_marker, vec_pose_init_ugv, "post_points_ugv_m", 5);	// RED: colour_ = 0 ; GREEN : colour_ = 1 ; BLUE: colour_ = 2 ; YELLOW = 3 ; PURPLE = 4 ; BLACK = 5 ; WHITE = 6;
	mp_.getMarkerLines(post_lines_ugv_marker, vec_pose_init_ugv, "post_lines_ugv_m", 5);
	post_traj_marker_ugv_pub_.publish(post_points_ugv_marker);
	post_traj_marker_ugv_pub_.publish(post_lines_ugv_marker);

}

void OptimizerLocalPlanner::calculateDistanceVertices(vector<double> &_v_D_ugv,vector<double> &_v_D_uav)
{
	float x, y, z;
	_v_D_ugv.clear();
	_v_D_uav.clear();
    double pL = 0.0;
	double sum_distance_ = 0.0;
	// fallowing lines count the number of UGV initial states in the same position
	// count_fix_points_ugv = count_not_fix_points_ugv = 0;
	// int aux_cont_ = 0;
    for (size_t i = 0; i < new_path_ugv.size()-1; i++)
    {
    	x = new_path_ugv[i+1].x() - new_path_ugv[i].x();
		y = new_path_ugv[i+1].y() - new_path_ugv[i].y();
		z = new_path_ugv[i+1].z() - new_path_ugv[i].z();
        pL= sqrt(x * x + y * y + z * z);
		// if (pL < 0.001){
		// 	count_not_fix_points_ugv = count_not_fix_points_ugv + 0;
		// 	if(aux_cont_ == i){ //To count the consecutive points that keep stable in the start point
		// 		count_fix_points_ugv = count_fix_points_ugv + 1;
		// 		aux_cont_++;
		// 	}
		// }
		// else
		// 	count_not_fix_points_ugv = count_not_fix_points_ugv + 1;

		sum_distance_ = sum_distance_ + pL;

		// printf("count_not_fix_points_ugv = %i  , sum_distance_ = %f , pl = %f \n",count_not_fix_points_ugv, sum_distance_,pL);
		// printf("[%lu] UGV : sum_distance_ = %f , pl = %f \n", i+1, sum_distance_, pL);
		_v_D_ugv.push_back(pL);
    }
	// ROS_INFO("sum_distance_=%f , new_path_ugv.size()=%lu , count_fix_points_ugv=%i",sum_distance_, new_path_ugv.size(), count_fix_points_ugv);
	initial_distance_states_ugv = sum_distance_ / ((double)(new_path_ugv.size()-(count_fix_points_ugv+1))); // Line when UGV initial states are considered in the same position
	// initial_distance_states_ugv = sum_distance_ / ((double)count_not_fix_points_ugv); // Line when UGV initial states are considered in the same position
	// initial_distance_states_ugv = sum_distance_ / (new_path_ugv.size()-1); // Line when UGV initial states are NOT considered in the same position

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
	double sum_T_ugv = 0.0;
	double sum_T_uav = 0.0;
	double _dT = 0;
	min_T_ugv = 1000.0;
	for (size_t i= 0 ; i < new_path_ugv.size(); i++){
		if (i == 0){
			_dT = 0.0;
			_time_ugv.push_back(_dT);
		}
		else{
			_dT = (vec_dist_init_ugv[i-1])/initial_velocity_ugv;
			_time_ugv.push_back(_dT);
			if (min_T_ugv > _dT)
				min_T_ugv = _dT;
		}
		sum_T_ugv = sum_T_ugv + _dT;
	}
	_dT = 0;
	for (size_t i= 0 ; i < new_path_uav.size(); i++){
		if (i == 0){
			_time_uav.push_back(0.0);
		}
		else{
			if( vec_dist_init_uav[i-1] < 0.00001)
				_dT = 0.0;
			else
				_dT = (vec_dist_init_uav[i-1])/initial_velocity_uav;
			_time_uav.push_back(_dT );
		}
		sum_T_uav = sum_T_uav + _dT;
	}
	initial_time_ugv = sum_T_ugv/ (new_path_ugv.size() - 1.0);
	initial_time_uav = sum_T_uav/ (new_path_uav.size() - 1.0);
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

	return ret;
}

void OptimizerLocalPlanner::publishOptimizedTraj(vector<Eigen::Vector3d> pos_ugv_opt_, vector<Eigen::Vector3d> pos_uav_opt_)
{
    ROS_INFO_COND(debug, "Publishing Trajectories");

	trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_;
	trajectory_msgs::MultiDOFJointTrajectory trajectory_;

    trajectory_.points.clear();

	trajectory_point_.transforms.resize(2);
	trajectory_point_.velocities.resize(2);
	trajectory_point_.accelerations.resize(2);

	for (size_t i = 0; i < pos_ugv_opt_.size() ; i++){
		trajectory_point_.transforms[0].translation.x = pos_ugv_opt_[i].x();
		trajectory_point_.transforms[0].translation.y = pos_ugv_opt_[i].y();
		trajectory_point_.transforms[0].translation.z = pos_ugv_opt_[i].z();
		trajectory_point_.transforms[0].rotation.x = statesRotUGV[i].parameter[1];
		trajectory_point_.transforms[0].rotation.y = statesRotUGV[i].parameter[2];
		trajectory_point_.transforms[0].rotation.z = statesRotUGV[i].parameter[3];
		trajectory_point_.transforms[0].rotation.w = statesRotUGV[i].parameter[4];
		if ( (i==0) || (vec_time_ugv_opt[i] < 0.0001) ){
			trajectory_point_.velocities[0].linear.x = 0.0;
			trajectory_point_.velocities[0].linear.y = 0.0;
			trajectory_point_.velocities[0].linear.z = 0.0;
		}
		else{ 
			trajectory_point_.velocities[0].linear.x = (pos_ugv_opt_[i+1].x() - pos_ugv_opt_[i].x())/vec_time_ugv_opt[i];
			trajectory_point_.velocities[0].linear.y = (pos_ugv_opt_[i+1].y() - pos_ugv_opt_[i].y())/vec_time_ugv_opt[i];
			trajectory_point_.velocities[0].linear.z = (pos_ugv_opt_[i+1].z() - pos_ugv_opt_[i].z())/vec_time_ugv_opt[i];
		}

		if ( (i==0) || (i==pos_ugv_opt_.size()-1) || (vec_time_ugv_opt[i] < 0.0001) ){
			trajectory_point_.accelerations[0].linear.x = 0.0;
			trajectory_point_.accelerations[0].linear.y = 0.0;
			trajectory_point_.accelerations[0].linear.z = 0.0;
		}
		else{
			trajectory_point_.accelerations[0].linear.x = 0.0;
			trajectory_point_.accelerations[0].linear.y = 0.0;
			trajectory_point_.accelerations[0].linear.z = 0.0;
		}

		trajectory_point_.transforms[1].translation.x = pos_uav_opt_[i].x();
		trajectory_point_.transforms[1].translation.y = pos_uav_opt_[i].y();
		trajectory_point_.transforms[1].translation.z = pos_uav_opt_[i].z();
		trajectory_point_.transforms[1].rotation.x = statesRotUAV[i].parameter[1];
		trajectory_point_.transforms[1].rotation.y = statesRotUAV[i].parameter[2];
		trajectory_point_.transforms[1].rotation.z = statesRotUAV[i].parameter[3];
		trajectory_point_.transforms[1].rotation.w = statesRotUAV[i].parameter[4];
		if ( (i==0) || (vec_time_uav_opt[i] < 0.0001) ){
			trajectory_point_.velocities[1].linear.x = 0.0;
			trajectory_point_.velocities[1].linear.y = 0.0;
			trajectory_point_.velocities[1].linear.z = 0.0;
		}
		else{ 
			trajectory_point_.velocities[1].linear.x = (pos_uav_opt_[i+1].x() - pos_uav_opt_[i].x())/vec_time_uav_opt[i];
			trajectory_point_.velocities[1].linear.y = (pos_uav_opt_[i+1].y() - pos_uav_opt_[i].y())/vec_time_uav_opt[i];
			trajectory_point_.velocities[1].linear.z = (pos_uav_opt_[i+1].z() - pos_uav_opt_[i].z())/vec_time_uav_opt[i];
		}

		if ( (i==0) || (i==pos_ugv_opt_.size()-1) || (vec_time_ugv_opt[i] < 0.0001) ){
			trajectory_point_.accelerations[1].linear.x = 0.0;
			trajectory_point_.accelerations[1].linear.y = 0.0;
			trajectory_point_.accelerations[1].linear.z = 0.0;
		}
		else{
			trajectory_point_.accelerations[1].linear.x = 0.0;
			trajectory_point_.accelerations[1].linear.y = 0.0;
			trajectory_point_.accelerations[1].linear.z = 0.0;
		}

		trajectory_.header.stamp = ros::Time::now();
	
		trajectory_.points.push_back(trajectory_point_);

	}

    trajPub.publish(trajectory_);
}

void OptimizerLocalPlanner::getKinematicTrajectory(vector<Eigen::Vector3d> v_pos2kin_ugv, vector<Eigen::Vector3d> v_pos2kin_uav, 
												   vector<double> &v_angles_kin_ugv, vector<double> &v_angles_kin_uav)
{
	v_angles_kin_ugv.clear();
	v_angles_kin_uav.clear();

	// Kinematics for ugv XY Axes
	for (size_t i=0 ; i < v_pos2kin_ugv.size()-2 ; i++){
		geometry_msgs::Point v1_ugv, v2_ugv;
		double dot_product_ugv, norm_vector1_ugv, norm_vector2_ugv, arg_norm_vector1_ugv, arg_norm_vector2_ugv, angle_ugv;
		v1_ugv.x=(v_pos2kin_ugv[i+1].x()-v_pos2kin_ugv[i].x());
		v1_ugv.y=(v_pos2kin_ugv[i+1].y()-v_pos2kin_ugv[i].y()); 
		v1_ugv.z=(v_pos2kin_ugv[i+1].z()-v_pos2kin_ugv[i].z());
		v2_ugv.x=(v_pos2kin_ugv[i+2].x()-v_pos2kin_ugv[i+1].x());
		v2_ugv.y=(v_pos2kin_ugv[i+2].y()-v_pos2kin_ugv[i+1].y()); 
		v2_ugv.z=(v_pos2kin_ugv[i+2].z()-v_pos2kin_ugv[i+1].z());
		dot_product_ugv = (v2_ugv.x * v1_ugv.x) + (v2_ugv.y * v1_ugv.y) + (v2_ugv.z * v2_ugv.z);
		arg_norm_vector1_ugv = (v1_ugv.x * v1_ugv.x) + (v1_ugv.y * v1_ugv.y) + (v2_ugv.z * v2_ugv.z);
		arg_norm_vector2_ugv = (v2_ugv.x * v2_ugv.x) + (v2_ugv.y * v2_ugv.y) + (v2_ugv.z * v2_ugv.z);

		if (arg_norm_vector1_ugv < 0.0001 && arg_norm_vector1_ugv > -0.0001 || arg_norm_vector2_ugv < 0.0001 && arg_norm_vector2_ugv > -0.0001)
			angle_ugv = 0.0;
		else{
			norm_vector1_ugv = sqrt(arg_norm_vector1_ugv);
			norm_vector2_ugv = sqrt(arg_norm_vector2_ugv);
			if ((dot_product_ugv / (norm_vector1_ugv*norm_vector2_ugv)) <= 1.000 )
				angle_ugv = acos(dot_product_ugv / (norm_vector1_ugv*norm_vector2_ugv));
			else
				angle_ugv = 0.000;
		}
		// ROS_INFO("dot_product_ugv= %f , norm_vector1_ugv= %f , norm_vector2_ugv = %f , angle_ugv= %f", dot_product_ugv, norm_vector1_ugv, norm_vector2_ugv,angle_ugv);
		v_angles_kin_ugv.push_back(angle_ugv);
	}

	for (size_t i=0 ; i < v_pos2kin_uav.size()-2 ; i++){
		geometry_msgs::Point v1_uav, v2_uav;
		double dot_product_uav, norm_vector1_uav, norm_vector2_uav, angle_uav;
		v1_uav.x=(v_pos2kin_uav[i+1].x()-v_pos2kin_uav[i].x());
		v1_uav.y=(v_pos2kin_uav[i+1].y()-v_pos2kin_uav[i].y()); 
		v1_uav.z=(v_pos2kin_uav[i+1].z()-v_pos2kin_uav[i].z());
		v2_uav.x=(v_pos2kin_uav[i+2].x()-v_pos2kin_uav[i+1].x());
		v2_uav.y=(v_pos2kin_uav[i+2].y()-v_pos2kin_uav[i+1].y()); 
		v2_uav.z=(v_pos2kin_uav[i+2].z()-v_pos2kin_uav[i+1].z());
		dot_product_uav = (v2_uav.x * v1_uav.x) + (v2_uav.y * v1_uav.y) + (v2_uav.z * v2_uav.z);
		norm_vector1_uav = sqrt((v1_uav.x * v1_uav.x) + (v1_uav.y * v1_uav.y) + (v2_uav.z * v2_uav.z) );
		norm_vector2_uav = sqrt((v2_uav.x * v2_uav.x) + (v2_uav.y * v2_uav.y) + (v2_uav.z * v2_uav.z) );

		if (norm_vector1_uav < 0.001 && norm_vector1_uav > -0.001 || norm_vector2_uav < 0.001 && norm_vector2_uav > -0.001)
			angle_uav = 0.0;
		else
			angle_uav = acos(dot_product_uav / (norm_vector1_uav*norm_vector2_uav));

		v_angles_kin_uav.push_back(angle_uav);
	}

}

void OptimizerLocalPlanner::postProcessingCatenary(){
	// Staff for fost-proccessing of catenary length
	parameterBlockLength parameter_block_post_length;
	statesPostLength.clear();
	int count_cat_fail = 0;
	dm_.openWriteCatenaryFailData();
	for(size_t i = 0; i < statesPosUAV.size(); i++){
		// Post-processing of catenary length
		parameter_block_post_length.parameter[0] = i; //id parameter
		double _d_ = sqrt(pow(statesPosUGV[i].parameter[1] - statesPosUAV[i].parameter[1],2) + 
						  pow(statesPosUGV[i].parameter[2] - statesPosUAV[i].parameter[2],2) + 
						  pow((statesPosUGV[i].parameter[3]+pos_reel_z) - statesPosUAV[i].parameter[3],2));
		if (_d_ > statesLength[i].parameter[1]){
			parameter_block_post_length.parameter[1] = _d_ * 1.005;	// 1.02 in catenary constraint
			count_cat_fail ++;
			dm_.writeCatenaryFailData(count_cat_fail,statesPosUGV[i].parameter[0],statesLength[i].parameter[1],_d_, parameter_block_post_length.parameter[1]);
		}
		else
			parameter_block_post_length.parameter[1] = statesLength[i].parameter[1];	
		
		statesPostLength.push_back(parameter_block_post_length);
	}
	statesLength.clear();
	statesLength = statesPostLength;
	dm_.closeWriteCatenaryFailData();
}