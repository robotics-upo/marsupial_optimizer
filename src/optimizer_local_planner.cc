/*
Optimizer Local Planner based in Ceres Solver for Marsupial Robotics COnfiguration 
Simon Martinez Rozas, 2020 
Service Robotics Lab, University Pablo de Olavide , Seville, Spain 
 */

#include "marsupial_optimizer/optimizer_local_planner.h"

OptimizerLocalPlanner::OptimizerLocalPlanner(bool get_path_from_file_)
{
	nh.reset(new ros::NodeHandle("~"));

    tfBuffer.reset(new tf2_ros::Buffer);
    tf2_list.reset(new tf2_ros::TransformListener(*tfBuffer));
    tf_list_ptr.reset(new tf::TransformListener(ros::Duration(5)));

	resetFlags();

	nh->param<double>("map_resolution", map_resolution,0.05);
	nh->param<double>("ws_x_max", ws_x_max, 10.0);
    nh->param<double>("ws_y_max", ws_y_max, 10.0);
    nh->param<double>("ws_z_max", ws_z_max, 20.0);
    nh->param<double>("ws_x_min", ws_x_min, -10.0);
    nh->param<double>("ws_y_min", ws_y_min, -10.0);
    nh->param<double>("ws_z_min", ws_z_min, 0.00);

    nh->param("world_frame", world_frame, (string) "/map");
	nh->param("ugv_base_frame", ugv_base_frame, (std::string) "ugv_base_link");
    nh->param("uav_base_frame", uav_base_frame, (std::string) "uav_base_link");
    nh->param("reel_base_frame", reel_base_frame, (std::string) "reel_base_link");

	nh->param<bool>("optimize_ugv",optimize_ugv, true);
	nh->param<bool>("optimize_uav",optimize_uav, true);
	nh->param<bool>("optimize_cat",optimize_cat, true);
	nh->param<bool>("fix_last_position_ugv",fix_last_position_ugv, false);
	nh->param<bool>("use_loss_function",use_loss_function, false);

	nh->param<bool>("equidistance_ugv_constraint", equidistance_ugv_constraint, true);
	nh->param<bool>("obstacles_ugv_constraint", obstacles_ugv_constraint, true);
	nh->param<bool>("traversability_ugv_constraint", traversability_ugv_constraint, true);
	nh->param<bool>("smoothness_ugv_constraint", smoothness_ugv_constraint, true);
	nh->param<bool>("velocity_ugv_constraint", velocity_ugv_constraint, true);
	nh->param<bool>("acceleration_ugv_constraint", acceleration_ugv_constraint, true);

	nh->param<bool>("time_constraint", time_constraint, true);

	nh->param<bool>("traj_in_rviz", traj_in_rviz, false); 
	nh->param<bool>("pause_end_optimization", pause_end_optimization, false);

	nh->param<bool>("equidistance_uav_constraint",equidistance_uav_constraint, true);
	nh->param<bool>("obstacles_uav_constraint",obstacles_uav_constraint, true);
	nh->param<bool>("smoothness_uav_constraint",smoothness_uav_constraint, true);
	nh->param<bool>("velocity_uav_constraint",velocity_uav_constraint, true);
	nh->param<bool>("acceleration_uav_constraint",acceleration_uav_constraint, true);

	nh->param<bool>("parable_obstacle_constraint",parable_obstacle_constraint, false);
	nh->param<bool>("parable_length_constraint",parable_length_constraint, false);
	nh->param<bool>("parable_parameters_constraint",parable_parameters_constraint, false);

	nh->param<double>("w_alpha_ugv", w_alpha_ugv,0.1);
	nh->param<double>("w_alpha_uav", w_alpha_uav,0.1);
	nh->param<double>("w_beta_uav",	w_beta_uav,0.1);
	nh->param<double>("w_beta_ugv",	w_beta_ugv,0.1);
	nh->param<double>("w_gamma_uav", w_gamma_uav,0.1);
	nh->param<double>("w_gamma_ugv", w_gamma_ugv,0.1);
	nh->param<double>("w_epsilon_ugv", w_epsilon_ugv,0.1);
	nh->param<double>("w_epsilon_uav", w_epsilon_uav,0.1);
	nh->param<double>("w_zeta_uav", w_zeta_uav,0.1);
	nh->param<double>("w_zeta_ugv", w_zeta_ugv,0.1);
	nh->param<double>("w_delta", w_delta,0.1);
	nh->param<double>("w_theta_ugv",	w_theta_ugv,0.1);

	nh->param<double>("w_kappa_ugv", w_kappa_ugv,0.1);
	nh->param<double>("w_kappa_uav", w_kappa_uav,0.1);
	
	nh->param<double>("w_mu_uav", w_mu_uav,0.1);
	nh->param<double>("w_nu_ugv", w_nu_ugv,0.1);
	
	nh->param<double>("w_eta_1", w_eta_1,0.1);
	nh->param<double>("w_eta_2", w_eta_2,0.1);
	nh->param<double>("w_eta_3", w_eta_3,0.1);

	nh->param<int>("count_fix_points_initial_ugv", count_fix_points_initial_ugv,1);
	nh->param<int>("n_iter_opt", n_iter_opt,200);
	nh->param<double>("distance_obstacle_ugv", distance_obstacle_ugv,0.5);
	nh->param<double>("distance_obstacle_uav", distance_obstacle_uav,1.0);
	nh->param<double>("initial_velocity_ugv", initial_velocity_ugv,1.0);
	nh->param<double>("initial_velocity_uav", initial_velocity_uav,1.0);
	nh->param<double>("initial_acceleration_ugv", initial_acceleration_ugv,0.0);
	nh->param<double>("initial_acceleration_uav", initial_acceleration_uav,0.0);
	nh->param<double>("angle_min_traj", angle_min_traj, M_PI / 15.0);
	nh->param<double>("distance_tether_obstacle", distance_tether_obstacle, 0.1);
	nh->param<double>("dynamic_catenary", dynamic_catenary, 0.5);
	nh->param<double>("length_tether_max", length_tether_max,20.0);

	nh->param<bool>("write_data_residual",write_data_residual, false);
	nh->param<bool>("verbose_optimizer",verbose_optimizer, true);
	nh->param<bool>("write_data_for_analysis",write_data_for_analysis, false);
	nh->param("path", path, (std::string) "~/");
	nh->param("path_mission_file", path_mission_file, (std::string) "~/missions/cfg/");
	nh->param("pc_user_name", user_name, (std::string) "simon");
	nh->param("files_results", files_results, (std::string) "results/");
	nh->param("files_residuals", files_residuals, (std::string) "residuals/");
	nh->param("name_output_file", name_output_file, (std::string) "optimization_test");
	nh->param("scenario_name", scenario_name, (std::string) "scenario_name");
	nh->param<int>("num_pos_initial", num_pos_initial, 1);

	nh->param<bool>("debug", debug, false);
 	nh->param<bool>("show_config", showConfig, false);
 	nh->param<bool>("use_distance_function", use_distance_function, true);
 	nh->param<bool>("use_parable", use_parable, true);

	ROS_INFO_COND(showConfig, PRINTF_BLUE "Optimizer Local Planner 3D Node Configuration:\n");
		
	step = map_resolution;
	step_inv = 1.0 / step;
	finished_rviz_maneuver = false;

	count_fix_points_final_ugv = count_fix_points_uav = count_fix_points_initial_ugv;

	initializeSubscribers();
	initializePublishers();
	setupOptimizer();
	cleanVectors();
	MP.clearMarkersPointLines(points_ugv_marker, lines_ugv_marker,traj_marker_ugv_pub_,20);
  	MP.clearMarkersPointLines(points_uav_marker, lines_uav_marker,traj_marker_uav_pub_,20);
	ROS_INFO(PRINTF_BLUE"Optimizer_Local_Planner: use_distance_function: %s",use_distance_function?"true":"false");
	ROS_INFO(PRINTF_BLUE"Optimizer_Local_Planner: alpha_uav=[%f] alpha_ugv=[%f] beta_uav=[%f] beta_ugv=[%f] theta_ugv=[%f] gamma_uav=[%f] gamma_ugv=[%f] kappa_ugv=[%f] kappa_uav=[%f] delta=[%f] epsilon=[%f] zeta=[%f] eta=[%f %f %f]",
						 w_alpha_uav, w_alpha_ugv, w_beta_uav, w_beta_ugv, w_theta_ugv,w_gamma_uav, w_gamma_ugv, w_kappa_ugv, 
						 w_kappa_uav, w_delta, w_epsilon_uav, w_zeta_uav, w_eta_1, w_eta_2, w_eta_3);
	
	std::string node_name_ = "grid3D_optimizer_node";
	grid_3D = new Grid3d(node_name_);
	grid_3D->computeTrilinearInterpolation();

	CheckCM = new CatenaryCheckerManager(node_name_);

	get_path_from_file = get_path_from_file_;
	if (!get_path_from_file){
		ROS_INFO(PRINTF_BLUE"Optimizer_Local_Planner: Parameters loaded, ready for Optimization Process. Waiting for Action!!");
		configServices();
	}
	else{
		ROS_INFO(PRINTF_BLUE"Optimizer_Local_Planner: Parameters loaded, ready for Optimization Process. Waiting for read path!!");
	}
}

void OptimizerLocalPlanner::initializeSubscribers()
{
	octomap_ws_sub_ = nh->subscribe( "/octomap_point_cloud_centers", 1,  &OptimizerLocalPlanner::readOctomapCallback, this);
    local_map_sub = nh->subscribe<octomap_msgs::Octomap>("/octomap_binary_local", 1, &OptimizerLocalPlanner::collisionMapCallBack, this);
    local_trav_map_sub = nh->subscribe<octomap_msgs::Octomap>("/oct_trav/octomap_binary", 1, &OptimizerLocalPlanner::traversableMapCallBack, this);
    point_cloud_ugv_obstacles_sub_ = nh->subscribe( "/region_growing_obstacles_pc_map", 1,  &OptimizerLocalPlanner::readPointCloudObstaclesUGVCallback, this);
    point_cloud_ugv_traversability_sub_ = nh->subscribe( "/region_growing_traversability_pc_map", 1,  &OptimizerLocalPlanner::readPointCloudTraversabilityUGVCallback, this);
    clean_markers_sub_ = nh->subscribe( "/clean_marker_optimizer", 1,  &OptimizerLocalPlanner::deleteMarkersCallBack, this);
    finished_rviz_maneuver_sub_ = nh->subscribe( "/finished_rviz_maneuver", 1,  &OptimizerLocalPlanner::finishedRvizManeuverCallBack, this);
    star_optimizer_process_sub_ = nh->subscribe( "/star_optimizer_process", 1,  &OptimizerLocalPlanner::initializeOptimizerProcessCallBack, this);

	ROS_INFO(PRINTF_BLUE"Optimizer_Local_Planner: Subscribers Initialized");
}

void OptimizerLocalPlanner::initializePublishers()
{
  	traj_marker_ugv_pub_ = nh->advertise<visualization_msgs::MarkerArray>("init_trajectory_ugv_marker", 2);
  	traj_marker_uav_pub_ = nh->advertise<visualization_msgs::MarkerArray>("init_trajectory_uav_marker", 2);
  	traj_opt_marker_ugv_pub_ = nh->advertise<visualization_msgs::MarkerArray>("opt_trajectory_uav_marker", 2);
  	traj_opt_marker_uav_pub_ = nh->advertise<visualization_msgs::MarkerArray>("opt_trajectory_uav_marker", 2);
	catenary_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("catenary_marker", 100);
	parable_marker_init_pub_ = nh->advertise<visualization_msgs::MarkerArray>("init_parable_marker", 200);
	parable_marker_opt_pub_ = nh->advertise<visualization_msgs::MarkerArray>("opt_parable_marker", 200);
    clean_nodes_marker_gp_pub_ = nh->advertise<std_msgs::Bool>("/clean_nodes_marker_gp", 1);
    clean_catenary_marker_gp_pub_ = nh->advertise<std_msgs::Bool>("/clean_catenary_marker_gp", 1);
	trajectory_pub_ = nh->advertise<marsupial_optimizer::marsupial_trajectory_optimized>("/trajectory_optimized", 200);

  	ROS_INFO(PRINTF_BLUE"Optimizer_Local_Planner: Publishers Initialized");
}

void OptimizerLocalPlanner::resetFlags()
{
  	mapReceivedFull = false;
  	mapReceivedTrav = false;
}

void OptimizerLocalPlanner::cleanVectors()
{
	vec_rot_ugv_init.clear(); 
	vec_rot_uav_init.clear();
	vec_len_cat_init.clear();
	vec_pose_ugv_init.clear();
	vec_pose_uav_init.clear();
	vec_dist_init_ugv.clear();
	vec_dist_init_uav.clear();
	vec_time_init.clear();
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
	options.num_threads = 1;
}

void OptimizerLocalPlanner::dynRecCb(marsupial_optimizer::OptimizationParamsConfig &config, uint32_t level)
{
    // ROS_INFO("Reconfigure Request: pause_end:%s , traj_in_rviz: %s",  pause_end_optimization.bool_param?"True":"False", traj_in_rviz.bool_param?"True":"False");
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
    pc_obs_ugv = msg;
	nn_ugv_obs.setInput(*msg);
  	ROS_INFO_COND(debug, PRINTF_BLUE "Local Planner: Received Point Cloud for KDTree obstacles UGV");
}

void OptimizerLocalPlanner::collisionMapCallBack(const octomap_msgs::OctomapConstPtr &msg)
{
    mapReceivedFull = true;
	mapFull_msg = (octomap::OcTree *)octomap_msgs::binaryMsgToMap(*msg);
}

void OptimizerLocalPlanner::traversableMapCallBack(const octomap_msgs::OctomapConstPtr &msg)
{
    mapReceivedTrav = true;
	mapTrav_msg = (octomap::OcTree *)octomap_msgs::binaryMsgToMap(*msg);
}

void OptimizerLocalPlanner::deleteMarkersCallBack(const std_msgs::BoolConstPtr &msg)
{
	if (msg->data == true){
		MP.clearMarkers(catenary_marker, 150, catenary_marker_pub_);
		MP.clearMarkersPointLines(points_ugv_marker, lines_ugv_marker,traj_marker_ugv_pub_,30);
  		MP.clearMarkersPointLines(points_uav_marker, lines_uav_marker,traj_marker_uav_pub_,30);
	}
}

void OptimizerLocalPlanner::finishedRvizManeuverCallBack(const std_msgs::BoolConstPtr &msg)
{
	if (msg->data == true)
		finished_rviz_maneuver = msg->data;
}

void OptimizerLocalPlanner::executeOptimizerPathPreemptCB()
{
    ROS_INFO_COND(debug, "Goal Preempted");
    execute_path_srv_ptr->setPreempted(); // set the action state to preempted

    resetFlags();
    MP.clearMarkersPointLines(points_ugv_marker, lines_ugv_marker, traj_marker_ugv_pub_, 0);
    MP.clearMarkersPointLines(points_uav_marker, lines_uav_marker, traj_marker_uav_pub_, 0);
}

void OptimizerLocalPlanner::initializeOptimizerProcessCallBack(const std_msgs::BoolConstPtr &msg)
{
	bool start_optimizer_process = msg->data;
	if (start_optimizer_process == true){
		executeOptimizerPathGoalCB();
	}
	start_optimizer_process = false;
}

void OptimizerLocalPlanner::executeOptimizerPathGoalCB()
{
  	ROS_INFO(PRINTF_GREEN "Optimizer Local Planner : Path received in action server mode\n");

	if (!get_path_from_file){
		auto path_shared_ptr = execute_path_srv_ptr->acceptNewGoal();
		globalPath = path_shared_ptr->path;
		vec_len_cat_init = path_shared_ptr->length_catenary; 
	} else{
		upo_actions::ExecutePathGoal path_shared_;
		ManagePath mp_(path+"results_marsupial_optimizer/rrt_path_2023_6_20_234860.yaml", path_shared_);
		globalPath = path_shared_.path;
		vec_len_cat_init = path_shared_.length_catenary; 
	}
  	
	if(write_data_residual){
		dm_.cleanResidualConstraintsFile(path, files_residuals);
	}
    getReelPose(); // To get init pos reel for optimization process

	CheckCM->Init(grid_3D, distance_tether_obstacle, distance_obstacle_ugv, distance_obstacle_uav, length_tether_max, ws_z_min, step, 
	use_parable, use_distance_function, pose_reel_local.transform.translation);
	// Stage to interpolate path
	InterpolatePath ip_;
	ip_.initInterpolatePath(count_fix_points_initial_ugv,count_fix_points_final_ugv,count_fix_points_uav, fix_last_position_ugv, distance_tether_obstacle,
							use_distance_function, globalPath.points.size(), length_tether_max, ws_z_min, map_resolution, 
							pose_reel_local, grid_3D);

	ip_.getInitialGlobalPath(globalPath, vec_len_cat_init, vec_pose_ugv_init, vec_pose_uav_init, vec_rot_ugv_init, vec_rot_uav_init);
	size_path = vec_pose_uav_init.size();

	checkCatenaryLength(vec_pose_ugv_init, vec_pose_uav_init, vec_rot_ugv_init, vec_len_cat_init);

	// Clear optimized Markers
	MP.clearMarkers(catenary_marker, 150, catenary_marker_pub_);
  	MP.clearMarkersPointLines(points_ugv_marker, lines_ugv_marker,traj_marker_ugv_pub_,size_path);
  	MP.clearMarkersPointLines(points_uav_marker, lines_uav_marker,traj_marker_uav_pub_,size_path);

	calculateDistanceVertices(vec_dist_init_ugv, vec_dist_init_uav);
 	getTemporalState(vec_time_init);

	// Stage to get parable
	getParableParameter(vec_pose_ugv_init, vec_pose_uav_init, vec_len_cat_init);
	graphCatenary(vec_pose_ugv_init,vec_pose_uav_init, vec_rot_ugv_init, vec_len_cat_init);
	graphParableAndPathMarker(vec_pose_ugv_init,vec_pose_uav_init, vec_rot_ugv_init, v_parable_params_init, 5, 6, 2,
							  traj_marker_ugv_pub_, traj_marker_uav_pub_, parable_marker_init_pub_, parable_marker_init);

    CheckCM->CheckStatusCollision(vec_pose_ugv_init, vec_rot_ugv_init, vec_pose_uav_init, v_parable_params_init);

	dm_.initDataManagement(path+files_results, name_output_file, scenario_name, num_pos_initial, initial_velocity_ugv, initial_velocity_uav, 
							initial_acceleration_ugv, initial_acceleration_uav, distance_tether_obstacle, pose_reel_local.transform.translation, vec_pose_ugv_init, 
							vec_pose_uav_init, vec_len_cat_init, vec_rot_ugv_init, vec_rot_uav_init, mapFull_msg, mapTrav_msg, grid_3D);			   
	if(write_data_for_analysis)
		dm_.getSmoothnessTrajectory(vec_pose_ugv_init, vec_pose_uav_init, v_angles_smooth_ugv_init, v_angles_smooth_uav_init);
		dm_.writeTemporalDataBeforeOptimization(vec_dist_init_ugv, vec_dist_init_uav, vec_time_init, v_angles_smooth_ugv_init, v_angles_smooth_uav_init, v_parable_params_init);

	std::cout << std::endl <<  "==================================================="  << std::endl << "\tPreparing to execute Optimization: Creating Parameter Blocks!!" << std::endl;

  	// Initialize optimizer
  	Problem problem;
	// Initializing parameters block to optimization
    initializingParametersblock();
	LossFunction* loss_function = NULL;
	if(use_loss_function){
		loss_function = new CauchyLoss(0.5);
	}

	/********************* To obligate pause method and check Planning result *********************/
                //    std::string w_ ;
                //    std::cout << " *** Press key to continue optimization process: " << std::endl;
                //    std::cin >> w_ ;
    /*************************************************************************************************/
	// Initializing Contraints for optimization	
	/****************************   UGV Constraints  ****************************/	
	if (optimize_ugv){
		ROS_INFO(PRINTF_GREEN" PREPARING  UGV  PARAMETER  BLOCKS  TO  OPTIMIZE:");
		if (initial_distance_states_ugv < 0.001){
			count_fix_points_initial_ugv = statesPosUGV.size();
			initial_distance_states_ugv = 0.001;
		}
		/*** Cost Function UGV I : Equidistance constraint UGV ***/
		if (equidistance_ugv_constraint){
			ROS_INFO(PRINTF_GREEN"		- Optimize Equidistance Autodiff");
			for (int i = 0; i <  statesPosUGV.size() - 1 ; ++i) {
				CostFunction* cost_function_ugv_1  = new AutoDiffCostFunction<EquiDistanceFunctorUGV, 1, 4, 4>
												(new EquiDistanceFunctorUGV(w_alpha_ugv, initial_distance_states_ugv,write_data_residual, user_name)); 
				problem.AddResidualBlock(cost_function_ugv_1, loss_function, statesPosUGV[i].parameter, statesPosUGV[i+1].parameter);
				if (i < count_fix_points_initial_ugv)
					problem.SetParameterBlockConstant(statesPosUGV[i].parameter);
				if (i > (statesPosUGV.size() - (count_fix_points_final_ugv+2)))
					problem.SetParameterBlockConstant(statesPosUGV[i+1].parameter);
				if (i == (statesPosUGV.size() - 2)) //To make fixed last UGV position
					problem.SetParameterBlockConstant(statesPosUGV[i+1].parameter);
				problem.SetParameterLowerBound(statesPosUGV[i].parameter, 1, ws_x_min);
				problem.SetParameterLowerBound(statesPosUGV[i].parameter, 2, ws_y_min);
				problem.SetParameterLowerBound(statesPosUGV[i].parameter, 3, ws_z_min);
				problem.SetParameterUpperBound(statesPosUGV[i].parameter, 1, ws_x_max);
				problem.SetParameterUpperBound(statesPosUGV[i].parameter, 2, ws_y_max);
				problem.SetParameterUpperBound(statesPosUGV[i].parameter, 3, ws_z_max);
			}
		}
		/*** Cost Function UGV II : Obstacles constraint UGV***/
		if (obstacles_ugv_constraint){
			ROS_INFO(PRINTF_GREEN"		- Optimize Obstacles Autodiff");
			for (int i = 0; i < statesPosUGV.size(); ++i) {
				CostFunction* cost_function_ugv_3  = new AutoDiffCostFunction<ObstacleDistanceFunctorUGV::ObstaclesFunctorUGV, 1, 4>
												(new ObstacleDistanceFunctorUGV::ObstaclesFunctorUGV(w_beta_ugv,distance_obstacle_ugv,
																									nn_ugv_obs.kdtree,nn_ugv_obs.obs_points,write_data_residual,user_name)); 
				problem.AddResidualBlock(cost_function_ugv_3, loss_function, statesPosUGV[i].parameter);
				if (i < count_fix_points_initial_ugv)
					problem.SetParameterBlockConstant(statesPosUGV[i].parameter);
				if (i > (statesPosUGV.size() - (count_fix_points_final_ugv+1)))
					problem.SetParameterBlockConstant(statesPosUGV[i].parameter);
			}	
		}	
		/*** Cost Function UGV III : Traversability constraint UGV***/
		if (traversability_ugv_constraint){
			ROS_INFO(PRINTF_GREEN"		- Optimize Traversability Autodiff");
			for (int i = 0; i < statesPosUGV.size(); ++i) {
				CostFunction* cost_function_ugv_4  = new AutoDiffCostFunction<TraversabilityDistanceFunctorUGV::TraversabilityFunctor, 1, 4>
												(new TraversabilityDistanceFunctorUGV::TraversabilityFunctor(w_theta_ugv, nn_trav.kdtree, nn_trav.obs_points, write_data_residual,user_name)); 
				problem.AddResidualBlock(cost_function_ugv_4, loss_function, statesPosUGV[i].parameter);
				if (i < count_fix_points_initial_ugv)
					problem.SetParameterBlockConstant(statesPosUGV[i].parameter);
				if (i > (statesPosUGV.size() - (count_fix_points_final_ugv+1)))
					problem.SetParameterBlockConstant(statesPosUGV[i].parameter);
			}
		}
		/*** Cost Function UGV IV : Smoothness constrain UGV ***/
		if (smoothness_ugv_constraint){
			ROS_INFO(PRINTF_GREEN"		- Optimize Smoothness Autodiff");
			for (int i = 0; i < statesPosUGV.size() - 2; ++i) {
				if ( (i >= count_fix_points_initial_ugv)  && i<(statesPosUGV.size() - (count_fix_points_final_ugv+1))){
					CostFunction* cost_function_ugv_5  = new AutoDiffCostFunction<SmoothnessFunctorUGV, 1, 4, 4, 4>(new SmoothnessFunctorUGV(w_gamma_ugv, angle_min_traj,write_data_residual,user_name)); 
					problem.AddResidualBlock(cost_function_ugv_5, loss_function, statesPosUGV[i].parameter, statesPosUGV[i+1].parameter, statesPosUGV[i+2].parameter);
				}
			}
		}
		/*** Cost Function UGV VI : Time constraint UGV***/
		if (time_constraint){
			ROS_INFO(PRINTF_GREEN"		- Optimize Time Autodiff");
			for (int i = 0; i < statesTime.size(); ++i) {
				CostFunction* cost_function_time  = new AutoDiffCostFunction<TimeFunctor, 1, 2> 
													(new TimeFunctor(w_delta, initial_time, write_data_residual, user_name)); 
				problem.AddResidualBlock(cost_function_time, loss_function, statesTime[i].parameter);
				if (i < 1) 
					problem.SetParameterBlockConstant(statesTime[i].parameter);
				else
					problem.SetParameterLowerBound(statesTime[i].parameter, 1, min_T);
			}
		}	
		/*** Cost Function UGV VII : Velocity constraint UGV***/
		if (velocity_ugv_constraint){
			ROS_INFO(PRINTF_GREEN"		- Optimize Velocity Autodiff");
			for (int i = 0; i < statesTime.size() - 1; ++i) {
				CostFunction* cost_function_ugv_8  = new AutoDiffCostFunction<VelocityFunctorUGV, 1, 4, 4, 2>
													(new VelocityFunctorUGV(w_epsilon_ugv, initial_velocity_ugv, count_fix_points_initial_ugv,write_data_residual,user_name)); 
				problem.AddResidualBlock(cost_function_ugv_8, loss_function, statesPosUGV[i].parameter, statesPosUGV[i+1].parameter, statesTime[i+1].parameter);
			}
		}
		/*** Cost Function UGV VIII : Acceleration constraint UGV***/
		if (acceleration_ugv_constraint){
			ROS_INFO(PRINTF_GREEN"		- Optimize Acceleration Autodiff");
			for (int i = 0; i < statesTime.size() - 2; ++i) {
				CostFunction* cost_function_ugv_9  = new AutoDiffCostFunction<AccelerationFunctorUGV, 1, 4, 4, 4, 2, 2>
													(new AccelerationFunctorUGV(w_zeta_ugv, initial_acceleration_ugv, count_fix_points_initial_ugv,write_data_residual,user_name)); 
				problem.AddResidualBlock(cost_function_ugv_9, loss_function, statesPosUGV[i].parameter, statesPosUGV[i+1].parameter, statesPosUGV[i+2].parameter, 
																statesTime[i+1].parameter, statesTime[i+2].parameter);
			}
		}
	}
	/****************************   UAV Constraints  ****************************/	
	if (optimize_uav){
		ROS_INFO(PRINTF_BLUE" PREPARING  UAV  PARAMETER  BLOCKS  TO  OPTIMIZE:");
		int fix_pos_init_uav = -1;
		int fix_pos_final_uav = 1000;

		/*** Cost Function UAV I : Equidistance constraint UAV ***/
		if (equidistance_uav_constraint){
			ROS_INFO(PRINTF_BLUE"		- Optimize Equidistance Autodiff");
			for (int i = 0; i <  statesPosUAV.size() - 1 ; ++i) {
				CostFunction* cost_function_uav_1  = new AutoDiffCostFunction<EquiDistanceFunctorUAV, 1, 4 ,4>
												(new EquiDistanceFunctorUAV(w_alpha_uav, initial_distance_states_uav, write_data_residual, user_name)); 
				problem.AddResidualBlock(cost_function_uav_1, loss_function, statesPosUAV[i].parameter, statesPosUAV[i+1].parameter);
				if (i < count_fix_points_uav){ 
					problem.SetParameterBlockConstant(statesPosUAV[i].parameter);
					fix_pos_init_uav=i;
				}
				if (i == (statesPosUAV.size() - 2)){
					problem.SetParameterBlockConstant(statesPosUAV[i+1].parameter);
					fix_pos_final_uav=i+1;
				}
				problem.SetParameterLowerBound(statesPosUAV[i].parameter, 1, ws_x_min);
				problem.SetParameterLowerBound(statesPosUAV[i].parameter, 2, ws_y_min);
				problem.SetParameterLowerBound(statesPosUAV[i].parameter, 3, ws_z_min);
				problem.SetParameterUpperBound(statesPosUAV[i].parameter, 1, ws_x_max);
				problem.SetParameterUpperBound(statesPosUAV[i].parameter, 2, ws_y_max);
				problem.SetParameterUpperBound(statesPosUAV[i].parameter, 3, ws_z_max);
			}
		}
		/*** Cost Function UAV II : Obstacles constraint UAV***/
		if (obstacles_uav_constraint){
			ROS_INFO(PRINTF_BLUE"		- Optimize Obstacles Autodiff");
			for (int i = 0; i < statesPosUAV.size(); ++i) {
				CostFunction* cost_function_uav_2  = new AutoDiffCostFunction<ObstacleDistanceFunctorUAV::ObstaclesFunctor , 1, 4>
												(new ObstacleDistanceFunctorUAV::ObstaclesFunctor (w_beta_uav, distance_obstacle_uav, nn_uav.kdtree, nn_uav.obs_points, path+files_results, write_data_residual, grid_3D, use_distance_function, user_name)); 
				problem.AddResidualBlock(cost_function_uav_2, loss_function, statesPosUAV[i].parameter);
				if (i == 0)
					problem.SetParameterBlockConstant(statesPosUAV[i].parameter);
				if (i == (statesPosUAV.size() - 1))
					problem.SetParameterBlockConstant(statesPosUAV[i].parameter);
				problem.SetParameterLowerBound(statesPosUAV[i].parameter, 1, ws_x_min);
				problem.SetParameterLowerBound(statesPosUAV[i].parameter, 2, ws_y_min);
				problem.SetParameterLowerBound(statesPosUAV[i].parameter, 3, ws_z_min);
				problem.SetParameterUpperBound(statesPosUAV[i].parameter, 1, ws_x_max);
				problem.SetParameterUpperBound(statesPosUAV[i].parameter, 2, ws_y_max);
				problem.SetParameterUpperBound(statesPosUAV[i].parameter, 3, ws_z_max);
			}
		}
		/*** Cost Function UAV III : Smoothness constraint UAV ***/
		if (smoothness_uav_constraint){
			ROS_INFO(PRINTF_BLUE"		- Optimize Smoothness Autodiff");
			for (int i = 0; i < statesPosUAV.size() - 2; ++i) {
				CostFunction* cost_function_uav_4  = new AutoDiffCostFunction<SmoothnessFunctorUAV, 1, 4, 4, 4>
												 (new SmoothnessFunctorUAV(w_gamma_uav, angle_min_traj,write_data_residual,user_name)); 
				problem.AddResidualBlock(cost_function_uav_4, loss_function, statesPosUAV[i].parameter, statesPosUAV[i+1].parameter, statesPosUAV[i+2].parameter);
			}
		}
		/*** Cost Function UAV V : Velocity constraint UAV ***/
		if(velocity_uav_constraint){
			ROS_INFO(PRINTF_BLUE"		- Optimize Velocity Autodiff");
			for (int i = 0; i < statesTime.size() - 1; ++i) {
				CostFunction* cost_function_uav_7  = new AutoDiffCostFunction<VelocityFunctorUAV, 1, 4, 4, 2>
												(new VelocityFunctorUAV(w_epsilon_uav, initial_velocity_uav,write_data_residual, user_name)); 
				problem.AddResidualBlock(cost_function_uav_7, loss_function, 
										statesPosUAV[i].parameter, statesPosUAV[i+1].parameter, 
										statesTime[i+1].parameter);
			}	
		}
		/*** Cost Function UAV VI : Acceleration constraint UAV ***/
		if(acceleration_uav_constraint){
			ROS_INFO(PRINTF_BLUE"		- Optimize Acceleration Autodiff");
			for (int i = 0; i < statesTime.size() - 2; ++i) {
				CostFunction* cost_function_uav_8  = new AutoDiffCostFunction<AccelerationFunctorUAV, 1, 4, 4, 4, 2, 2>
												(new AccelerationFunctorUAV(w_zeta_uav, initial_acceleration_uav,write_data_residual, user_name)); 
				problem.AddResidualBlock(cost_function_uav_8, loss_function, 
										statesPosUAV[i].parameter, statesPosUAV[i+1].parameter, statesPosUAV[i+2].parameter, 
										statesTime[i+1].parameter, statesTime[i+2].parameter);
			}
		}
	}
	/****************************   Parable Constraints  ****************************/	
	if (optimize_cat){
		ROS_INFO(PRINTF_ORANGE" PREPARING  PARABLE  PARAMETER  BLOCKS  TO  OPTIMIZE:");
		/*** Cost Function Cable I : Parable constrain  ***/
		if(parable_obstacle_constraint){
			ROS_INFO(PRINTF_ORANGE"		- Optimize Parable Obstacles Autodiff");
			for (int i = 0; i < statesParableParams.size(); ++i) {
				CostFunction* cost_function_par_1  = new AutoDiffCostFunction<AutodiffParableFunctor::ParableFunctor, 1, 4, 4, 4> //Residual, ugvPos, uavPos, parableParams
											(new AutodiffParableFunctor::ParableFunctor(w_eta_1, grid_3D, pose_reel_local.transform.translation, distance_tether_obstacle,
											write_data_residual, user_name)); 
					problem.AddResidualBlock(cost_function_par_1, loss_function, statesPosUGV[i].parameter, statesPosUAV[i].parameter, statesParableParams[i].parameter);
				if (i == 0 || (fix_last_position_ugv && i == statesParableParams.size()-1))
					problem.SetParameterBlockConstant(statesParableParams[i].parameter);
			}
		}
		if(parable_length_constraint){
			ROS_INFO(PRINTF_ORANGE"		- Optimize Parable Length Autodiff");
			for (int i = 0; i < statesParableParams.size(); ++i) {
				CostFunction* cost_function_par_2  = new AutoDiffCostFunction<AutodiffParableLengthFunctor::ParableLengthFunctor, 1, 4, 4, 4>
											(new AutodiffParableLengthFunctor::ParableLengthFunctor(w_eta_2, pose_reel_local.transform.translation, 
											length_tether_max, write_data_residual, user_name)); 
					problem.AddResidualBlock(cost_function_par_2, loss_function, statesPosUGV[i].parameter, statesPosUAV[i].parameter, statesParableParams[i].parameter);
				if (i == 0 || (fix_last_position_ugv && i == statesParableParams.size()-1))
					problem.SetParameterBlockConstant(statesParableParams[i].parameter);
				else{
					problem.SetParameterLowerBound(statesParableParams[i].parameter, 1, 0.000001);
				}
			}		
    	}
		if(parable_parameters_constraint){
			ROS_INFO(PRINTF_ORANGE"		- Optimize Parable Params Autodiff");
			for (int i = 0; i < statesParableParams.size(); ++i) {
				CostFunction* cost_function_par_3  = new AutoDiffCostFunction<AutodiffParableParametersFunctor::ParableParametersFunctor, 2, 4, 4, 4>
											(new AutodiffParableParametersFunctor::ParableParametersFunctor(w_eta_3, pose_reel_local.transform.translation, 
											write_data_residual, user_name)); 
					problem.AddResidualBlock(cost_function_par_3, loss_function, statesPosUGV[i].parameter, statesPosUAV[i].parameter, statesParableParams[i].parameter);
				if (i == 0 || (fix_last_position_ugv && i == statesParableParams.size()-1))
					problem.SetParameterBlockConstant(statesParableParams[i].parameter);
				else{
					problem.SetParameterLowerBound(statesParableParams[i].parameter, 1, 0.000001);
				}
			}		
    	}
	}
	
	//Clean Nodes Markers Global Planner
	std_msgs::Bool clean_markers_gp_;
    clean_markers_gp_.data = true;
    clean_nodes_marker_gp_pub_.publish(clean_markers_gp_); 
    clean_catenary_marker_gp_pub_.publish(clean_markers_gp_); 

	std::cout <<  "==================================================="  << std::endl << "Optimization  started !!" << std::endl;

	start_time_opt = ros::Time::now();
	ceres::Solve(options, &problem, &summary);
	std::cout << summary.FullReport() << "\n";
	final_time_opt = ros::Time::now();
	initial_cost = summary.initial_cost;
	final_cost = summary.final_cost;
	successful_steps = summary.num_successful_steps;
	unsuccessful_steps = summary.num_unsuccessful_steps;
	time_optimazation = summary.total_time_in_seconds;

	finishigOptimization();
	// Inform if is a feabible Trajectory or not, and 
	
	// checkCollisionPathPlanner ccpp("grid3D_optimizer_node", grid_3D, pc_obs_ugv, pose_reel_local.transform.translation, distance_obstacle_ugv, distance_obstacle_uav, distance_tether_obstacle);
	// std::cout << "executeOptimizerPathGoalCB(): vec_pose_ugv_opt= " << vec_pose_ugv_opt.size() << 
	// 											" vec_rot_ugv_opt=" << vec_rot_ugv_opt.size() << 
	// 											" vec_pose_uav_opt=" << vec_pose_uav_opt.size() << 
	// 											" v_parable_params_opt=" << v_parable_params_opt.size() << std::endl;
    bool check_collision_ = CheckCM->CheckStatusCollision(vec_pose_ugv_opt, vec_rot_ugv_opt, vec_pose_uav_opt, v_parable_params_opt);

	// Initializing variable for optimization analysis
	if (write_data_for_analysis)
		writeDataForAnalysis(CheckCM->count_ugv_coll, CheckCM->count_uav_coll, CheckCM->count_tether_coll);
	
	double time_sleep_ = 2.0;
	bool finished_optimization_ = false;
	if (check_collision_){
		if (initial_cost==final_cost)
			ROS_INFO(PRINTF_ORANGE"\n		Optimizer Local Planner: Final Cost equal than Initial Cost");
		ROS_INFO(PRINTF_GREEN"\n\n\n		Optimizer Local Planner: Goal position successfully achieved through optimization trajectory\n\n\n");

	// Visualize traj in RVIZ
		if (traj_in_rviz){
			// publishOptimizedTraj();
			while(!finished_rviz_maneuver){
				ros::spinOnce();
				printf("Waiting to finish maneuver in Rviz\n");
			};
			ros::Duration(time_sleep_).sleep();
			finished_rviz_maneuver = false;
		}
		else{
			ros::Duration(time_sleep_).sleep();
		}
		action_result.arrived = true;

		finished_optimization_ = true;
	}
	else{
		if (initial_cost==final_cost)
				ROS_INFO(PRINTF_ORANGE"\n		Optimizer Local Planner: Final Cost equal than Initial Cost");
		ROS_INFO(PRINTF_RED"\n\n\n\n		Optimizer Local Planner: Goal position Not achieved through optimization trajectory\n\n\n");
		if (traj_in_rviz){
			// publishOptimizedTraj();
			ros::Duration(time_sleep_).sleep();
			while(!finished_rviz_maneuver){
				ros::spinOnce();
				printf("Waiting to finish maneuver in Rviz\n");
			};
			ros::Duration(time_sleep_).sleep();
		}
		else{
			ros::Duration(time_sleep_).sleep();
		}	
		finished_rviz_maneuver = false;
		action_result.arrived = false;

		finished_optimization_ = true;		
	}
	std::cout <<"Optimization Proccess Completed !!!" << std::endl << "Saving Temporal data in txt file ..." << std::endl << "===================================================" << std::endl << std::endl << std::endl;
    ros::Duration(2.0).sleep();

	/********************* To obligate pause method and check Planning result *********************/
        // std::string y_ ;
        // std::cout << " *** Press key to continue: " << std::endl;
        // std::cin >> y_ ;
    /*************************************************************************************************/

	cleanVectors();		//Clear vector after optimization and previus the next iteration
	// Clear optimized Markers
	
	MP.clearMarkers(catenary_marker, 150, parable_marker_init_pub_);
  	MP.clearMarkersPointLines(points_ugv_marker, lines_ugv_marker,traj_marker_ugv_pub_,size_path);
  	MP.clearMarkersPointLines(points_uav_marker, lines_uav_marker,traj_marker_uav_pub_,size_path);
	MP.clearMarkers(catenary_marker, 150, parable_marker_opt_pub_);
  	MP.clearMarkersPointLines(points_ugv_marker, lines_ugv_marker,traj_opt_marker_ugv_pub_,size_path);
  	MP.clearMarkersPointLines(points_uav_marker, lines_uav_marker,traj_opt_marker_uav_pub_,size_path);
	resetFlags();
	if (!get_path_from_file && finished_optimization_)
			execute_path_srv_ptr->setSucceeded(action_result);
}

void OptimizerLocalPlanner::initializingParametersblock()
{
	//Set Parameter Blocks from path global information
	statesPosUGV.clear(); statesPosUAV.clear(); statesRotUGV.clear(); statesRotUAV.clear(); statesTime.clear();
	statesTime.clear(); statesLength.clear(); statesParableParams.clear();

	
	printf("Initial States.Parameter[____] :   [  pos_x  pos_y  pos_z  rot_r  rot_p  rot_y pos_x  pos_y  pos_z  rot_x  rot_y  rot_z  rot_w  time_ugv  time_uav  param_p  param_q  param_r length]\n");
	for (size_t i = 0 ; i < vec_pose_uav_init.size(); i ++){
		parameterBlockPos parameter_block_pos_ugv;
		parameterBlockPos parameter_block_pos_uav;
		parameterBlockRot parameter_block_rot_ugv;
		parameterBlockRot parameter_block_rot_uav;
		parameterBlockTime parameter_block_time;
		
		/*** Parameters for UGV ***/
		// Position Parameters
		parameter_block_pos_ugv.parameter[0] = i; //id parameter
		parameter_block_pos_ugv.parameter[1] = vec_pose_ugv_init[i].x;
		parameter_block_pos_ugv.parameter[2] = vec_pose_ugv_init[i].y; 
		parameter_block_pos_ugv.parameter[3] = vec_pose_ugv_init[i].z; 
		// Rotation Parameters
		parameter_block_rot_ugv.parameter[0] = i; //id parameter 
		parameter_block_rot_ugv.parameter[1] = vec_rot_ugv_init[i].x;
		parameter_block_rot_ugv.parameter[2] = vec_rot_ugv_init[i].y; 
		parameter_block_rot_ugv.parameter[3] = vec_rot_ugv_init[i].z; 
		parameter_block_rot_ugv.parameter[4] = vec_rot_ugv_init[i].w;
		// Time Parameters
		parameter_block_time.parameter[0] = i; //id parameter 
		parameter_block_time.parameter[1] = vec_time_init[i];

		/*** Parameters for UAV ***/
		// Position Parameters
		parameter_block_pos_uav.parameter[0] = i; //id parameter
		parameter_block_pos_uav.parameter[1] = vec_pose_uav_init[i].x;
		parameter_block_pos_uav.parameter[2] = vec_pose_uav_init[i].y; 
		parameter_block_pos_uav.parameter[3] = vec_pose_uav_init[i].z; 
		//Rotation Parameters
		parameter_block_rot_uav.parameter[0] = i; //id parameter 
		parameter_block_rot_uav.parameter[1] = vec_rot_uav_init[i].x;
		parameter_block_rot_uav.parameter[2] = vec_rot_uav_init[i].y; 
		parameter_block_rot_uav.parameter[3] = vec_rot_uav_init[i].z; 
		parameter_block_rot_uav.parameter[4] = vec_rot_uav_init[i].w; 

		statesPosUGV.push_back(parameter_block_pos_ugv);
		statesPosUAV.push_back(parameter_block_pos_uav);
		statesRotUGV.push_back(parameter_block_rot_ugv);
		statesRotUAV.push_back(parameter_block_rot_uav);
		statesTime.push_back(parameter_block_time);

		//Convert quaternion to euler angles
		double roll_ugv_init_, pitch_ugv_init_, yaw_ugv_init_;
		tf::Quaternion q_ugv_init_(parameter_block_rot_ugv.parameter[1],parameter_block_rot_ugv.parameter[2],parameter_block_rot_ugv.parameter[3],parameter_block_rot_ugv.parameter[4]);
		tf::Matrix3x3 M_ugv_init_(q_ugv_init_);	
		M_ugv_init_.getRPY(roll_ugv_init_, pitch_ugv_init_, yaw_ugv_init_);

		printf("Initial States.Parameter[%lu/%lu]: UGV=[%.3f %.3f %.3f / %.3f %.3f %.3f]",i,vec_pose_uav_init.size(),parameter_block_pos_ugv.parameter[1],
				parameter_block_pos_ugv.parameter[2],parameter_block_pos_ugv.parameter[3], roll_ugv_init_, pitch_ugv_init_, yaw_ugv_init_);
		printf(" UAV=[%.3f %.3f %.3f / %.3f %.3f %.3f %.3f] / [%.3f %.3f] / [%.3f %.3f %.3f] / [%.3f]\n", parameter_block_pos_uav.parameter[1],parameter_block_pos_uav.parameter[2],
				parameter_block_pos_uav.parameter[3], parameter_block_rot_uav.parameter[1],parameter_block_rot_uav.parameter[2],parameter_block_rot_uav.parameter[3],
				parameter_block_rot_uav.parameter[4], parameter_block_time.parameter[1], parameter_block_time.parameter[1],v_parable_params_init[i].p, v_parable_params_init[i].q, v_parable_params_init[i].r, vec_len_cat_init[i])	;
	}
}

void OptimizerLocalPlanner::finishigOptimization()
{
	vec_pose_ugv_opt.clear(); vec_rot_ugv_opt.clear();
	vec_pose_uav_opt.clear(); vec_rot_uav_opt.clear();
	vec_time_opt.clear();
	v_parable_params_opt.clear();
	double new_equi_dist = 0; 
	double distance_ = 0;
  	for (size_t i = 0; i < size_path; i++){
		geometry_msgs::Vector3 position_ugv_, position_uav_;
		position_ugv_.x = statesPosUGV[i].parameter[1];
		position_ugv_.y = statesPosUGV[i].parameter[2];
		position_ugv_.z = statesPosUGV[i].parameter[3];
		vec_pose_ugv_opt.push_back(position_ugv_);
		position_uav_.x = statesPosUAV[i].parameter[1];
		position_uav_.y = statesPosUAV[i].parameter[2];
		position_uav_.z = statesPosUAV[i].parameter[3];
		vec_pose_uav_opt.push_back(position_uav_);

		parable_parameters param_; 
		param_.p = statesParableParams[i].parameter[1];
		param_.q = statesParableParams[i].parameter[2];
		param_.r = statesParableParams[i].parameter[3];
		v_parable_params_opt.push_back(param_);

		vec_time_opt.push_back(statesTime[i].parameter[1]);
		geometry_msgs::Quaternion rot_angle_ugv_, rot_angle_uav_;
		rot_angle_ugv_.x = statesRotUGV[i].parameter[1];
		rot_angle_ugv_.y = statesRotUGV[i].parameter[2];
		rot_angle_ugv_.z = statesRotUGV[i].parameter[3];
		rot_angle_ugv_.w = statesRotUGV[i].parameter[4];
		vec_rot_ugv_opt.push_back(rot_angle_ugv_);
		rot_angle_uav_.x = statesRotUAV[i].parameter[1];
		rot_angle_uav_.y = statesRotUAV[i].parameter[2];
		rot_angle_uav_.z = statesRotUAV[i].parameter[3];
		rot_angle_uav_.w = statesRotUAV[i].parameter[4];
		vec_rot_uav_opt.push_back(rot_angle_uav_);	
	}
	
	graphParableAndPathMarker(vec_pose_ugv_opt, vec_pose_uav_opt, vec_rot_ugv_opt, v_parable_params_opt, 1, 2, 3,
							  traj_opt_marker_ugv_pub_,traj_opt_marker_uav_pub_, parable_marker_opt_pub_, parable_marker_opt);

	ManagePath mp_;

	for(size_t i = 0; i < statesPosUAV.size(); i++){
		double dist_ = sqrt(pow(vec_pose_ugv_opt[i].x - vec_pose_uav_opt[i].x,2)+pow(vec_pose_ugv_opt[i].y - vec_pose_uav_opt[i].y,2)+pow(vec_pose_ugv_opt[i].z+0.5 - vec_pose_uav_opt[i].z,2));
		double length_ = checkParableLength(v_parable_params_opt[i], vec_pose_ugv_opt[i], vec_pose_uav_opt[i]);
		printf(PRINTF_REGULAR"Optimized States.Parameter[%lu/%lu]: UGV=[%.3f %.3f %.3f / %.3f %.3f %.3f %.3f] UAV=[%.3f %.3f %.3f / %.3f %.3f %.3f %.3f] , t=[%.3f/%.3f], parable=[%.3f %.3f %.3f] length=[%.3f/%.3f]\n", 
		i,statesPosUAV.size(),
		statesPosUGV[i].parameter[1], statesPosUGV[i].parameter[2], statesPosUGV[i].parameter[3],
		statesRotUGV[i].parameter[1], statesRotUGV[i].parameter[2], statesRotUGV[i].parameter[3], statesRotUGV[i].parameter[4],
		statesPosUAV[i].parameter[1], statesPosUAV[i].parameter[2], statesPosUAV[i].parameter[3],
		statesRotUAV[i].parameter[1], statesRotUAV[i].parameter[2], statesRotUAV[i].parameter[3], statesRotUAV[i].parameter[4],
		statesTime[i].parameter[1],statesTime[i].parameter[1], 
		v_parable_params_opt[i].p, v_parable_params_opt[i].q, v_parable_params_opt[i].r, 
		length_, dist_);
	}
}

void OptimizerLocalPlanner::writeDataForAnalysis(int ugv_coll_, int uav_coll_, int tether_coll_){ 
	v_angles_smooth_ugv_opt.clear();
	v_angles_smooth_uav_opt.clear();
	dm_.getSmoothnessTrajectory(vec_pose_ugv_opt, vec_pose_uav_opt, v_angles_smooth_ugv_opt, v_angles_smooth_uav_opt);
	dm_.writeTemporalDataAfterOptimization(size_path, 
										vec_pose_ugv_opt, 
										vec_pose_uav_opt, 
										vec_rot_ugv_opt, 
										vec_rot_uav_opt, 
										vec_time_opt, 
										v_parable_params_opt, 
										v_angles_smooth_ugv_opt, 
										v_angles_smooth_uav_opt);
	std::cout << "...Temporal data saved in txt file." << std::endl << std::endl;
	std::cout << "Saving data for analisys in txt file..." << std::endl;
	double opt_compute_time_ = (final_time_opt - start_time_opt).toSec(); //Compute Time Optimization
	dm_.getDataForOptimizerAnalysis(nn_ugv_obs.kdtree, nn_uav.kdtree, nn_ugv_obs.obs_points, nn_uav.obs_points, opt_compute_time_, "UGV");
	dm_.getDataForOptimizerAnalysis(nn_uav.kdtree,     nn_uav.kdtree, nn_uav.obs_points,     nn_uav.obs_points, opt_compute_time_, "UAV");
	dm_.feasibilityAnalisysTrajectory(initial_cost, final_cost, successful_steps, unsuccessful_steps, time_optimazation, ugv_coll_, uav_coll_, tether_coll_);
	std::cout << "... data for analysis saved in txt file." << std::endl << std::endl;
}

void OptimizerLocalPlanner::calculateDistanceVertices(vector<double> &_v_D_ugv,vector<double> &_v_D_uav)
{
	float x, y, z;
	_v_D_ugv.clear();
	_v_D_uav.clear();
    double pL = 0.0;
	double sum_distance_ = 0.0;

    for (size_t i = 0; i < vec_pose_ugv_init.size()-1; i++)
    {
    	x = vec_pose_ugv_init[i+1].x - vec_pose_ugv_init[i].x;
		y = vec_pose_ugv_init[i+1].y - vec_pose_ugv_init[i].y;
		z = vec_pose_ugv_init[i+1].z - vec_pose_ugv_init[i].z;
        pL= sqrt(x * x + y * y + z * z);
		sum_distance_ = sum_distance_ + pL;
		_v_D_ugv.push_back(pL);
    }
	double den_ = vec_pose_ugv_init.size()-(double) count_fix_points_initial_ugv;
	if (den_ <= 0.000001)
		initial_distance_states_ugv = 0.0;
	else
		initial_distance_states_ugv = sum_distance_ / den_; // Line when UGV initial states are considered in the same position

	pL = 0.0;
	sum_distance_ = 0.0;
	for (size_t i = 0; i < vec_pose_uav_init.size()-1; i++)
    {
    	x = vec_pose_uav_init[i+1].x - vec_pose_uav_init[i].x;
		y = vec_pose_uav_init[i+1].y - vec_pose_uav_init[i].y;
		z = vec_pose_uav_init[i+1].z - vec_pose_uav_init[i].z;
        pL= sqrt(x * x + y * y + z * z);
		
		sum_distance_ = sum_distance_ + pL;
		_v_D_uav.push_back(pL);
    }
	int aux_n_p = round(vec_pose_uav_init.size()*0.1);
	initial_distance_states_uav = sum_distance_ / (vec_pose_uav_init.size()+aux_n_p);
}

void OptimizerLocalPlanner::getTemporalState(vector<double> &_time) 
{
	_time.clear();
	double sum_T = 0.0;
	double _dT = 0;
	int count_=0;
	min_T = 1000.0;
	for (size_t i= 0 ; i < vec_pose_uav_init.size(); i++){
		if (i == 0){
			_time.push_back(0.0);
		}
		else{
			if( vec_dist_init_uav[i-1] > vec_dist_init_ugv[i-1] ){
				_dT = (vec_dist_init_uav[i-1])/initial_velocity_uav;
			}
			else
				_dT = (vec_dist_init_ugv[i-1])/initial_velocity_ugv;
			_time.push_back(_dT );
			if (min_T > _dT)
				min_T = _dT;
		}
		sum_T = sum_T + _dT;
	}
	initial_time = sum_T/ (vec_pose_uav_init.size() - 1.0);
}

void OptimizerLocalPlanner::getReelPose()
{
    try{
        pose_reel_global = tfBuffer->lookupTransform(world_frame, reel_base_frame, ros::Time(0));
    }catch (tf2::TransformException &ex){
        ROS_WARN("Optimizer Local Planner: Couldn't get Global Reel Pose (frame: %s), so not possible to set Tether start point; tf exception: %s", reel_base_frame.c_str(),ex.what());
    }

	try{
        pose_reel_local = tfBuffer->lookupTransform(ugv_base_frame, reel_base_frame, ros::Time(0));
		// printf("Optimizer: pose_reel_local = [%f %f %f]\n", pose_reel_local.transform.translation.x, pose_reel_local.transform.translation.y, pose_reel_local.transform.translation.z);
    }catch (tf2::TransformException &ex){
        ROS_WARN("Optimizer Local Planner: Couldn't get Local Reel Pose (frame: %s), so not possible to set Tether start point; tf exception: %s", reel_base_frame.c_str(),ex.what());
    }
}

geometry_msgs::Vector3 OptimizerLocalPlanner::getReelPoint(const float px_, const float py_, const float pz_,const float qx_, const float qy_, const float qz_, const float qw_)
{
	geometry_msgs::Vector3 ret;

	double roll_, pitch_, yaw_;
	tf::Quaternion q_(qx_,qy_,qz_,qw_);
	tf::Matrix3x3 M_(q_);	
	M_.getRPY(roll_, pitch_, yaw_);

	double lengt_vec =  sqrt(pose_reel_local.transform.translation.x*pose_reel_local.transform.translation.x + pose_reel_local.transform.translation.y*pose_reel_local.transform.translation.y);
	ret.x = px_ + lengt_vec *cos(yaw_); 
	ret.y = py_ + lengt_vec *sin(yaw_);
	ret.z = pz_ + pose_reel_local.transform.translation.z ;

	return ret;
}

void OptimizerLocalPlanner::getParableParameter(vector<geometry_msgs::Vector3> v_p_init_ugv_, vector<geometry_msgs::Vector3> v_p_init_uav_, vector<float> &v_l_cat_init_){

    GetParableParameter GPP_(v_p_init_ugv_, v_p_init_uav_, v_l_cat_init_, vec_rot_ugv_init, pose_reel_local);
	GPP_.ParametersParable();
	v_parable_params_init.clear();
	v_parable_params_init = GPP_.v_parable_params;

	parameterBlockParable parable_params_ ;
	for (size_t i = 0 ; i < GPP_.v_parable_params.size() ; i++){
		parable_params_.parameter[0] = i;
		parable_params_.parameter[1] = GPP_.v_parable_params[i].p;
		parable_params_.parameter[2] = GPP_.v_parable_params[i].q;
		parable_params_.parameter[3] = GPP_.v_parable_params[i].r;
		statesParableParams.push_back(parable_params_);
	}
}

void OptimizerLocalPlanner::graphCatenary(vector<geometry_msgs::Vector3> v_ugv_, vector<geometry_msgs::Vector3> v_uav_, vector<geometry_msgs::Quaternion> v_rot_ugv, vector<float>  v_cat_){
	bisectionCatenary bc;
	
	for(size_t i = 0; i < v_uav_.size(); i++){
		std::vector<geometry_msgs::Vector3> points_catenary_final;
		points_catenary_final.clear();
	  	// The Reel Position is consider above base_link_ugv
		geometry_msgs::Vector3 p_reel_ = getReelPoint(v_ugv_[i].x,v_ugv_[i].y,v_ugv_[i].z,v_rot_ugv[i].x, v_rot_ugv[i].y, v_rot_ugv[i].z, v_rot_ugv[i].w);
		bc.configBisection(v_cat_[i], p_reel_.x, p_reel_.y, p_reel_.z, v_uav_[i].x, v_uav_[i].y, v_uav_[i].z);
		bc.getPointCatenary3D(points_catenary_final, false);
		MP.markerPoints(catenary_marker, points_catenary_final, i, v_uav_.size(), catenary_marker_pub_,1);	
	}
}

void OptimizerLocalPlanner::graphParableAndPathMarker(vector<geometry_msgs::Vector3> v_ugv_, vector<geometry_msgs::Vector3> v_uav_, 
													  vector<geometry_msgs::Quaternion> v_rot_ugv, vector <parable_parameters> v_params_, 
													  int c_ugv_, int c_uav_, int c_parable_, ros::Publisher p_ugv_, ros::Publisher p_uav_, 
													  ros::Publisher p_parable_, visualization_msgs::MarkerArray m_){
	
	std::vector<geometry_msgs::Vector3> v_pts_parable_;
	geometry_msgs::Vector3 p_reel_;

	v_pts_parable_.clear();
	GetParableParameter GPP_;
	for(size_t i = 0; i < v_params_.size(); i++){ // The Reel Position is consider above base_link_ugv
		p_reel_ = getReelPoint(v_ugv_[i].x,v_ugv_[i].y,v_ugv_[i].z,v_rot_ugv[i].x, v_rot_ugv[i].y, v_rot_ugv[i].z, v_rot_ugv[i].w);
		GPP_.getParablePoints(p_reel_, v_uav_[i], v_params_[i], v_pts_parable_);
		MP.markerPoints(m_, v_pts_parable_, i, v_pts_parable_.size(), p_parable_, c_parable_,false);	

	// 	/********************* To obligate pause method and check Planning result *********************/
    //                std::string v_ ;
    //                std::cout << " *** Published Tether Marker ["<< i <<"]" << std::endl;
    //                std::cout << " *** Press key to continue tether publisher marker: " << std::endl;
    //                std::cin >> v_ ;
    // /*************************************************************************************************/
	}

	MP.getMarkerPoints(points_ugv_marker, v_ugv_, "points_ugv_m",c_ugv_);	// RED= 0 ; GREEN= 1 ; BLUE= 2 ; YELLOW= 3 ; PURPLE= 4; BLACK=5; WHITE=6
	MP.getMarkerLines(lines_ugv_marker, v_ugv_, "lines_ugv_m",c_ugv_);
	MP.getMarkerPoints(points_uav_marker, v_uav_, "points_uav_m",c_uav_);
	MP.getMarkerLines(lines_uav_marker, v_uav_, "lines_uav_m",c_uav_);
	p_ugv_.publish(points_ugv_marker);
	p_ugv_.publish(lines_ugv_marker);
	p_uav_.publish(points_uav_marker);
	p_uav_.publish(lines_uav_marker);
}

void OptimizerLocalPlanner::checkCatenaryLength(vector<geometry_msgs::Vector3> v_p_ugv, vector<geometry_msgs::Vector3>  v_p_uav, vector<geometry_msgs::Quaternion> v_r_ugv, vector<float> &v_l_in){

	geometry_msgs::Vector3 p_reel_;
	std::vector<float>  v_l_out;
	v_l_out.clear();

	for(size_t i = 0 ; i < v_l_in.size() ; i++){
		p_reel_ = getReelPoint(v_p_ugv[i].x , v_p_ugv[i].y , v_p_ugv[i].z,
                           	   v_r_ugv[i].x, v_r_ugv[i].y, v_r_ugv[i].z, v_r_ugv[i].w);
		double d_ = sqrt(pow(v_p_uav[i].x - p_reel_.x,2) + 
						 pow(v_p_uav[i].y - p_reel_.y,2) + 
						 pow(v_p_uav[i].z - p_reel_.z,2));
		if (d_ > v_l_in[i]){
			double length_corrected_ = d_ * 1.001;
			ROS_ERROR("OptimizerLocalPlanner::checkCatenaryLength : [%lu] ugv[%.3f %.3f %.3f] uav[%.3f %.3f %.3f] lenght_current=%f length_corrected = %f distance=%f ",
			i, v_p_ugv[i].x , v_p_ugv[i].y , v_p_ugv[i].z, v_p_uav[i].x , v_p_uav[i].y , v_p_uav[i].z, v_l_in[i] ,length_corrected_,d_);
			v_l_out.push_back(d_ * 1.001);
		}
		else
			v_l_out.push_back(v_l_in[i]);
			ROS_INFO("OptimizerLocalPlanner::checkCatenaryLength : [%lu] ugv[%.3f %.3f %.3f] uav[%.3f %.3f %.3f] lenght_current=%f  distance=%f ",
			i, v_p_ugv[i].x , v_p_ugv[i].y , v_p_ugv[i].z, v_p_uav[i].x , v_p_uav[i].y , v_p_uav[i].z, v_l_in[i] ,d_);
	}
	v_l_in.clear();       //Check This two lines, should be fixed in a previous method the length
	v_l_in = v_l_out;     //Check This two lines, should be fixed in a previous method the length
}

double OptimizerLocalPlanner::checkParableLength(parable_parameters p_, geometry_msgs::Vector3 p1_ , geometry_msgs::Vector3 p2_){

	double L1, L2, L, x_;
	// Compute parable L : log(q + ((q + 2*p*x)^2 + 1)^(1/2) + 2*p*x)/(4*p) + ((q + 2*p*x)*((q + 2*p*x)^2 + 1)^(1/2))/(4*p) , x = xA and xB
	x_ = 0.0;
	L1 = log(p_.q + sqrt(pow(p_.q + 2*p_.p*x_,2) + 1) + 2.0*p_.p*x_)/(4.0*p_.p) + ((p_.q + 2*p_.p*x_)*sqrt(pow(p_.q + 2.0*p_.p*x_,2) + 1))/(4.0*p_.p);
	x_ = sqrt(pow(p1_.x - p2_.x,2)+pow(p1_.y - p2_.y,2));
	L2 = log(p_.q + sqrt(pow(p_.q + 2*p_.p*x_,2) + 1) + 2.0*p_.p*x_)/(4.0*p_.p) + ((p_.q + 2*p_.p*x_)*sqrt(pow(p_.q + 2.0*p_.p*x_,2) + 1))/(4.0*p_.p);
	L = L2 - L1;

	return L;
}