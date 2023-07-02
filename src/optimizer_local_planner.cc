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

	nh->param<int>("equidistance_ugv_constraint", equidistance_ugv_constraint, 1);
	nh->param<int>("obstacles_ugv_constraint", obstacles_ugv_constraint, 1);
	nh->param<int>("traversability_ugv_constraint", traversability_ugv_constraint, 1);
	nh->param<int>("smoothness_ugv_constraint", smoothness_ugv_constraint, 1);
	nh->param<bool>("velocity_ugv_constraint", velocity_ugv_constraint, true);
	nh->param<bool>("acceleration_ugv_constraint", acceleration_ugv_constraint, true);

	nh->param<bool>("time_constraint", time_constraint, true);

	nh->param<bool>("traj_in_rviz", traj_in_rviz, false); 
	nh->param<bool>("pause_end_optimization", pause_end_optimization, false);

	nh->param<int>("equidistance_uav_constraint",equidistance_uav_constraint, 1);
	nh->param<int>("obstacles_uav_constraint",obstacles_uav_constraint, 1);
	nh->param<int>("smoothness_uav_constraint",smoothness_uav_constraint, 1);
	nh->param<bool>("velocity_uav_constraint",velocity_uav_constraint, true);
	nh->param<bool>("acceleration_uav_constraint",acceleration_uav_constraint, true);

	nh->param<int>("catenary_constraint",catenary_constraint, 0);
	nh->param<bool>("catenary_length_constraint",catenary_length_constraint, false);

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
	nh->param<int>("scenario_number", scenario_number, 1);
	nh->param<int>("num_pos_initial", num_pos_initial, 1);

	nh->param<bool>("debug", debug, false);
 	nh->param<bool>("show_config", showConfig, false);
 	nh->param<bool>("use_distance_function", use_distance_function, true);

	ROS_INFO_COND(showConfig, PRINTF_BLUE "Optimizer Local Planner 3D Node Configuration:\n");
		
	step = map_resolution;
	step_inv = 1.0 / step;
	finished_rviz_maneuver = false;

	initializeSubscribers();
	initializePublishers();
	setupOptimizer();
	cleanVectors();
	mp_.clearMarkersPointLines(points_ugv_marker, lines_ugv_marker,traj_marker_ugv_pub_,20);
  	mp_.clearMarkersPointLines(points_uav_marker, lines_uav_marker,traj_marker_uav_pub_,20);
	ROS_INFO(PRINTF_BLUE"Optimizer_Local_Planner: use_distance_function: %s",use_distance_function?"true":"false");
	ROS_INFO(PRINTF_BLUE"Optimizer_Local_Planner: alpha_uav=[%f] alpha_ugv=[%f] beta_uav=[%f] beta_ugv=[%f] theta_ugv=[%f] gamma_uav=[%f] gamma_ugv=[%f] kappa_ugv=[%f] kappa_uav=[%f] delta=[%f] epsilon=[%f] zeta=[%f] eta_1=[%f] eta_2=[%f]",
						 w_alpha_uav, w_alpha_ugv, w_beta_uav, w_beta_ugv, w_theta_ugv,w_gamma_uav, w_gamma_ugv, w_kappa_ugv, 
						 w_kappa_uav, w_delta, w_epsilon_uav, w_zeta_uav, w_eta_1, w_eta_2);
	
	std::string node_name_ = "grid3D_optimizer_node";
	grid_3D = new Grid3d(node_name_);
	grid_3D->computeTrilinearInterpolation();

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
  	post_traj_marker_ugv_pub_ = nh->advertise<visualization_msgs::MarkerArray>("post_init_trajectory_ugv_marker", 2);
	post_traj_marker_uav_pub_ = nh->advertise<visualization_msgs::MarkerArray>("post_init_trajectory_uav_marker", 2);
  	traj_marker_ugv_pub_ = nh->advertise<visualization_msgs::MarkerArray>("opt_trajectory_ugv_marker", 2);
  	traj_marker_uav_pub_ = nh->advertise<visualization_msgs::MarkerArray>("opt_trajectory_uav_marker", 2);
	catenary_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("catenary_marker", 100);
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
	vec_init_rot_ugv.clear(); 
	vec_init_rot_uav.clear();
	vec_len_cat_init.clear();
	vec_pose_init_ugv.clear();
	vec_pose_init_uav.clear();
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
		mp_.clearMarkers(catenary_marker, 150, catenary_marker_pub_);
		mp_.clearMarkersPointLines(points_ugv_marker, lines_ugv_marker,traj_marker_ugv_pub_,30);
  		mp_.clearMarkersPointLines(points_uav_marker, lines_uav_marker,traj_marker_uav_pub_,30);
  		mp_.clearMarkersPointLines(post_points_ugv_marker, post_lines_ugv_marker, post_traj_marker_ugv_pub_,30);
  		mp_.clearMarkersPointLines(post_points_uav_marker, post_lines_uav_marker, post_traj_marker_uav_pub_,30);
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
    mp_.clearMarkersPointLines(points_ugv_marker, lines_ugv_marker, traj_marker_ugv_pub_, 0);
    mp_.clearMarkersPointLines(points_uav_marker, lines_uav_marker, traj_marker_uav_pub_, 0);
	mp_.clearMarkersPointLines(post_points_ugv_marker, post_lines_ugv_marker, post_traj_marker_ugv_pub_,0);
	mp_.clearMarkersPointLines(post_points_uav_marker, post_lines_uav_marker, post_traj_marker_uav_pub_,0);
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
  	ROS_INFO_COND(debug, PRINTF_GREEN "Optimizer Local Planner : Path received in action server mode");

	if (!get_path_from_file){
		auto path_shared_ptr = execute_path_srv_ptr->acceptNewGoal();
		globalPath = path_shared_ptr->path;
		vec_len_cat_init = path_shared_ptr->length_catenary; 
	} else{
		upo_actions::ExecutePathGoal path_shared_	;
		ImportPath ip_(path+"results_marsupial_optimizer/rrt_path_2023_6_20_234860.yaml", path_shared_);
		globalPath = path_shared_.path;
		vec_len_cat_init = path_shared_.length_catenary; 
	}
  	
	if(write_data_residual)
		cleanResidualConstraintsFile();
    getReelPose(); // To get init pos reel for optimization process

	// Stage to interpolate path
	InterpolatePath ip_;
	ip_.initInterpolatePath(count_fix_points_initial_ugv,count_fix_points_final_ugv,count_fix_points_uav, fix_last_position_ugv, distance_catenary_obstacle,
							use_distance_function, globalPath.points.size(), length_tether_max, ws_z_min, map_resolution, 
							pose_reel_local, grid_3D);

	ip_.getInitialGlobalPath(globalPath, vec_len_cat_init, vec_pose_init_ugv, vec_pose_init_uav, vec_init_rot_ugv, vec_init_rot_uav);
	size_path = vec_pose_init_uav.size();

	// Clear optimized Markers
	mp_.clearMarkers(catenary_marker, 150, catenary_marker_pub_);
  	mp_.clearMarkersPointLines(points_ugv_marker, lines_ugv_marker,traj_marker_ugv_pub_,size_path);
  	mp_.clearMarkersPointLines(points_uav_marker, lines_uav_marker,traj_marker_uav_pub_,size_path);
	mp_.clearMarkersPointLines(post_points_ugv_marker, post_lines_ugv_marker, post_traj_marker_ugv_pub_,size_path);
	mp_.clearMarkersPointLines(post_points_uav_marker, post_lines_uav_marker, post_traj_marker_uav_pub_,size_path);
	
	calculateDistanceVertices(vec_dist_init_ugv, vec_dist_init_uav);
 	getTemporalState(vec_time_init);

	dm_.initDataManagement(path+files_results, name_output_file, scenario_number, num_pos_initial, initial_velocity_ugv, initial_velocity_uav, 
							initial_acceleration_ugv, initial_acceleration_uav, distance_catenary_obstacle, pose_reel_local.transform.translation, vec_pose_init_ugv, 
							vec_pose_init_uav, vec_len_cat_init, vec_init_rot_ugv, vec_init_rot_uav, mapFull_msg, mapTrav_msg, grid_3D);			   
	if(write_data_for_analysis)
		getSmoothnessTrajectory(vec_pose_init_ugv, vec_pose_init_uav, v_init_angles_smooth_ugv, v_init_angles_smooth_uav);
		dm_.writeTemporalDataBeforeOptimization(vec_dist_init_ugv, vec_dist_init_uav, vec_time_init, v_init_angles_smooth_ugv, v_init_angles_smooth_uav);

	// Stage to get parable
	getParableParameter(vec_pose_init_ugv, vec_pose_init_uav, vec_len_cat_init);

	std::cout << std::endl <<  "==================================================="  << std::endl << "\tPreparing to execute Optimization: Creating Parameter Blocks!!" << std::endl;

  	// Initialize optimizer
  	Problem problem;

	// Initializing parameters block to optimization
    initializingParametersblock();

	// Initializing Contraints for optimization	
	/****************************   UGV Constraints  ****************************/	
	if (optimize_ugv){
		ROS_INFO(PRINTF_GREEN" PREPARING  UGV  PARAMETER  BLOCKS  TO  OPTIMIZE:");
		/*** Cost Function UGV I : Equidistance constraint UGV ***/
		if (equidistance_ugv_constraint==1){
			ROS_INFO(PRINTF_GREEN"		- Optimize Equidistance Autodiff");
			for (int i = 0; i <  statesPosUGV.size() - 1 ; ++i) {
				CostFunction* cost_function_ugv_1  = new AutoDiffCostFunction<EquiDistanceFunctorUGV, 1, 4, 4>
												(new EquiDistanceFunctorUGV(w_alpha_ugv, initial_distance_states_ugv,write_data_residual, user_name)); 
				problem.AddResidualBlock(cost_function_ugv_1, NULL, statesPosUGV[i].parameter, statesPosUGV[i+1].parameter);
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
		if (equidistance_ugv_constraint==0){
			ROS_INFO(PRINTF_GREEN"		- Optimize Equidistance Analytic");
			for (int i = 0; i <  statesPosUGV.size() - 1 ; ++i) {
				CostFunction* cost_function_ugv_1 = new EquiDistanceFunctorUGVAnalytic(w_alpha_ugv, initial_distance_states_ugv);
				problem.AddResidualBlock(cost_function_ugv_1, new ceres::CauchyLoss(0.1), statesPosUGV[i].parameter, statesPosUGV[i+1].parameter);
				if (i < count_fix_points_initial_ugv)
					problem.SetParameterBlockConstant(statesPosUGV[i].parameter);
				problem.SetParameterLowerBound(statesPosUGV[i].parameter, 3, 0.0); 
			}
		}
		/*** Cost Function UGV II : Obstacles constraint UGV***/
		if (obstacles_ugv_constraint==1){
			ROS_INFO(PRINTF_GREEN"		- Optimize Obstacles Autodiff");
			for (int i = 0; i < statesPosUGV.size(); ++i) {
				CostFunction* cost_function_ugv_3  = new AutoDiffCostFunction<ObstacleDistanceFunctorUGV::ObstaclesFunctorUGV, 1, 4>
												(new ObstacleDistanceFunctorUGV::ObstaclesFunctorUGV(w_beta_ugv,distance_obstacle_ugv,
																									nn_ugv_obs.kdtree,nn_ugv_obs.obs_points,write_data_residual,user_name)); 
				problem.AddResidualBlock(cost_function_ugv_3, NULL, statesPosUGV[i].parameter);
				if (i < count_fix_points_initial_ugv)
					problem.SetParameterBlockConstant(statesPosUGV[i].parameter);
				if (i > (statesPosUGV.size() - (count_fix_points_final_ugv+1)))
					problem.SetParameterBlockConstant(statesPosUGV[i].parameter);
			}	
		}
		if (obstacles_ugv_constraint==0){
			ROS_INFO(PRINTF_GREEN"		- Optimize Obstacles Analytic");
			for (int i = 0; i < statesPosUGV.size(); ++i) {
				CostFunction* cost_function_ugv_3 = new ObstacleDistanceFunctorUGVAnalytic(w_beta_ugv, distance_obstacle_ugv, count_fix_points_initial_ugv, 
																				nn_ugv_obs.kdtree,nn_ugv_obs.obs_points);
				problem.AddResidualBlock(cost_function_ugv_3, new ceres::CauchyLoss(0.1), statesPosUGV[i].parameter); 
			}
		}
		/*** Cost Function UGV III : Traversability constraint UGV***/
		if (traversability_ugv_constraint == 1){
			ROS_INFO(PRINTF_GREEN"		- Optimize Traversability Autodiff");
			for (int i = 0; i < statesPosUGV.size(); ++i) {
				CostFunction* cost_function_ugv_4  = new AutoDiffCostFunction<TraversabilityDistanceFunctorUGV::TraversabilityFunctor, 1, 4>
												(new TraversabilityDistanceFunctorUGV::TraversabilityFunctor(w_theta_ugv, nn_trav.kdtree, nn_trav.obs_points, write_data_residual,user_name)); 
				problem.AddResidualBlock(cost_function_ugv_4, NULL, statesPosUGV[i].parameter);
				if (i < count_fix_points_initial_ugv)
					problem.SetParameterBlockConstant(statesPosUGV[i].parameter);
				if (i > (statesPosUGV.size() - (count_fix_points_final_ugv+1)))
					problem.SetParameterBlockConstant(statesPosUGV[i].parameter);
			}
		}
		if (traversability_ugv_constraint == 0){
			ROS_INFO(PRINTF_GREEN"		- Optimize Traversability Analytic");
			for (int i = 0; i < statesPosUGV.size(); ++i) {
				CostFunction* cost_function_ugv_4 = new TraversabilityDistanceFunctorUGVAnalytic(w_theta_ugv, count_fix_points_initial_ugv, nn_trav.kdtree, nn_trav.obs_points);
				problem.AddResidualBlock(cost_function_ugv_4, new ceres::CauchyLoss(0.1), statesPosUGV[i].parameter); 
			}
		}
		/*** Cost Function UGV IV : Smoothness constrain UGV ***/
		if (smoothness_ugv_constraint == 1){
			ROS_INFO(PRINTF_GREEN"		- Optimize Smoothness Autodiff");
			for (int i = 0; i < statesPosUGV.size() - 2; ++i) {
				if ( (i >= count_fix_points_initial_ugv)  && i<(statesPosUGV.size() - (count_fix_points_final_ugv+1))){
					CostFunction* cost_function_ugv_5  = new AutoDiffCostFunction<SmoothnessFunctorUGV, 1, 4, 4, 4>(new SmoothnessFunctorUGV(w_gamma_ugv, angle_min_traj,write_data_residual,user_name)); 
					problem.AddResidualBlock(cost_function_ugv_5, NULL, statesPosUGV[i].parameter, statesPosUGV[i+1].parameter, statesPosUGV[i+2].parameter);
				}
			}
		}
		if (smoothness_ugv_constraint == 0){
			ROS_INFO(PRINTF_GREEN"		- Optimize Smoothness Analytic");
			for (int i = 0; i < statesPosUGV.size() - 2; ++i) {
				CostFunction* cost_function_ugv_5 = new smoothnessFunctorUGVAnalytic(w_gamma_ugv, angle_min_traj, count_fix_points_initial_ugv);
				problem.AddResidualBlock(cost_function_ugv_5, new ceres::CauchyLoss(0.1), statesPosUGV[i].parameter, statesPosUGV[i+1].parameter, statesPosUGV[i+2].parameter); 
			}
		}
		/*** Cost Function UGV V : Rotation constraint UGV ***/
		// for (int i = 0; i < statesRotUGV.size() - 1; ++i) {
		// 	double _roll, _pitch, _yaw;
		// 	tf::Quaternion _q(statesRotUGV[i+1].parameter[1],statesRotUGV[i+1].parameter[2],statesRotUGV[i+1].parameter[3],statesRotUGV[i+1].parameter[4]);
		// 	tf::Matrix3x3 _M(_q);	
		// 	_M.getRPY(_roll, _pitch, _yaw);
		// 	CostFunction* cost_function_ugv_6  = new NumericDiffCostFunction<RotationFunctorUGV, CENTRAL, 1, 4, 4, 5>(new RotationFunctorUGV(w_kappa_ugv,_yaw)); 
		// 	problem.AddResidualBlock(cost_function_ugv_6, NULL, statesPosUGV[i].parameter, statesPosUGV[i+1].parameter, statesRotUGV[i+1].parameter);
		// }

		/*** Cost Function UGV VI : Time constraint UGV***/
		if (time_constraint){
			ROS_INFO(PRINTF_GREEN"		- Optimize Time Autodiff");
			for (int i = 0; i < statesTime.size(); ++i) {
				CostFunction* cost_function_time  = new AutoDiffCostFunction<TimeFunctor, 1, 2> (new TimeFunctor(w_delta, initial_time, write_data_residual, user_name)); 
				problem.AddResidualBlock(cost_function_time, NULL, statesTime[i].parameter);
				if (i < count_fix_points_initial_ugv) 
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
				problem.AddResidualBlock(cost_function_ugv_8, NULL, statesPosUGV[i].parameter, statesPosUGV[i+1].parameter, statesTime[i+1].parameter);
			}
		}
		/*** Cost Function UGV VIII : Acceleration constraint UGV***/
		if (acceleration_ugv_constraint){
			ROS_INFO(PRINTF_GREEN"		- Optimize Acceleration Autodiff");
			for (int i = 0; i < statesTime.size() - 2; ++i) {
				CostFunction* cost_function_ugv_9  = new AutoDiffCostFunction<AccelerationFunctorUGV, 1, 4, 4, 4, 2, 2>
													(new AccelerationFunctorUGV(w_zeta_ugv, initial_acceleration_ugv, count_fix_points_initial_ugv,write_data_residual,user_name)); 
				problem.AddResidualBlock(cost_function_ugv_9, NULL, statesPosUGV[i].parameter, statesPosUGV[i+1].parameter, statesPosUGV[i+2].parameter, 
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
		if (equidistance_uav_constraint==1){
			ROS_INFO(PRINTF_BLUE"		- Optimize Equidistance Autodiff");
			for (int i = 0; i <  statesPosUAV.size() - 1 ; ++i) {
				CostFunction* cost_function_uav_1  = new AutoDiffCostFunction<EquiDistanceFunctorUAV, 1, 4 ,4>
												(new EquiDistanceFunctorUAV(w_alpha_uav, initial_distance_states_uav, write_data_residual, user_name)); 
				problem.AddResidualBlock(cost_function_uav_1, NULL, statesPosUAV[i].parameter, statesPosUAV[i+1].parameter);
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
		if (equidistance_uav_constraint==0){
			ROS_INFO(PRINTF_BLUE"		- Optimize Equidistance Analytic");
			for (int i = 0; i <  statesPosUAV.size() - 1 ; ++i) {
				CostFunction* cost_function_uav_1 = new EquiDistanceFunctorUAVAnalytic(w_alpha_uav, initial_distance_states_uav, size_path );
				problem.AddResidualBlock(cost_function_uav_1, new ceres::CauchyLoss(0.1), statesPosUAV[i].parameter, statesPosUAV[i+1].parameter);
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
		if(obstacles_uav_constraint==1){
			ROS_INFO(PRINTF_BLUE"		- Optimize Obstacles Autodiff");
			for (int i = 0; i < statesPosUAV.size(); ++i) {
				CostFunction* cost_function_uav_2  = new AutoDiffCostFunction<ObstacleDistanceFunctorUAV::ObstaclesFunctor , 1, 4>
												(new ObstacleDistanceFunctorUAV::ObstaclesFunctor (w_beta_uav, distance_obstacle_uav, nn_uav.kdtree, nn_uav.obs_points, path+files_results, write_data_residual, grid_3D, use_distance_function, user_name)); 
				problem.AddResidualBlock(cost_function_uav_2, NULL, statesPosUAV[i].parameter);
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
		if(obstacles_uav_constraint==0){
			ROS_INFO(PRINTF_BLUE"		- Optimize Obstacles Analytic");
			for (int i = 0; i < statesPosUAV.size(); ++i) {
				CostFunction* cost_function_uav_2 = new ObstacleDistanceFunctorUAVAnalytic(w_beta_uav, distance_obstacle_uav, grid_3D);
				problem.AddResidualBlock(cost_function_uav_2, new ceres::CauchyLoss(0.1), statesPosUAV[i].parameter); 
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
		if(smoothness_uav_constraint==1){
			ROS_INFO(PRINTF_BLUE"		- Optimize Smoothness Autodiff");
			for (int i = 0; i < statesPosUAV.size() - 2; ++i) {
				CostFunction* cost_function_uav_4  = new AutoDiffCostFunction<SmoothnessFunctorUAV, 1, 4, 4, 4>
												 (new SmoothnessFunctorUAV(w_gamma_uav, angle_min_traj,write_data_residual,user_name)); 
				problem.AddResidualBlock(cost_function_uav_4, NULL, statesPosUAV[i].parameter, statesPosUAV[i+1].parameter, statesPosUAV[i+2].parameter);
			}
		}	
		if(smoothness_uav_constraint==0){
			ROS_INFO(PRINTF_BLUE"		- Optimize Smoothness Analytic");
			for (int i = 0; i < statesPosUAV.size() - 2; ++i) {
				CostFunction* cost_function_uav_4 = new smoothnessFunctorUAVAnalytic(w_gamma_uav, angle_min_traj, fix_pos_init_uav, fix_pos_final_uav);
				problem.AddResidualBlock(cost_function_uav_4, new ceres::CauchyLoss(0.1), statesPosUAV[i].parameter, statesPosUAV[i+1].parameter, statesPosUAV[i+2].parameter);
			}
		}	

		/*** Cost Function UAV IV : Rotation constraint UAV ***/
		// for (int i = 0; i < statesRotUAV.size() - 1; ++i) {
		// 	CostFunction* cost_function_uav_5  = new NumericDiffCostFunction<RotationFunctorUAV, CENTRAL, 1, 5, 5>(new RotationFunctorUAV(w_kappa_uav)); 
		// 	problem.AddResidualBlock(cost_function_uav_5, NULL, statesRotUAV[i].parameter, statesRotUAV[i+1].parameter);
		// }
		// if(time_constraint){
		// 	ROS_INFO(PRINTF_BLUE"		- Optimize Time Autodiff");
		// 	/*** Cost Function UAV VI : Time constraint UAV***/
		// 	for (int i = 0; i < statesTimeUAV.size(); ++i) {
		// 		CostFunction* cost_function_uav_6  = new AutoDiffCostFunction<TimeFunctor, 1, 2>
		// 										(new TimeFunctor(w_delta, initial_time_uav)); 
		// 		problem.AddResidualBlock(cost_function_uav_6, NULL, statesTimeUAV[i].parameter);
		// 		if (i == 0)
		// 			problem.SetParameterBlockConstant(statesTimeUAV[i].parameter);
		// 		else
		// 			problem.SetParameterLowerBound(statesTimeUAV[i].parameter, 1, min_T_uav);
		// 	}
		// }

		/*** Cost Function UAV V : Velocity constraint UAV ***/
		if(velocity_uav_constraint){
			ROS_INFO(PRINTF_BLUE"		- Optimize Velocity Autodiff");
			for (int i = 0; i < statesTime.size() - 1; ++i) {
				CostFunction* cost_function_uav_7  = new AutoDiffCostFunction<VelocityFunctorUAV, 1, 4, 4, 2>
												(new VelocityFunctorUAV(w_epsilon_uav, initial_velocity_uav,write_data_residual, user_name)); 
				problem.AddResidualBlock(cost_function_uav_7, NULL, 
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
				problem.AddResidualBlock(cost_function_uav_8, NULL, 
										statesPosUAV[i].parameter, statesPosUAV[i+1].parameter, statesPosUAV[i+2].parameter, 
										statesTime[i+1].parameter, statesTime[i+2].parameter);
			}
		}
	}
	
	/****************************   Catenary Constraints  ****************************/	
	if (optimize_cat){
		ROS_INFO(PRINTF_ORANGE" PREPARING  CATENARY  PARAMETER  BLOCKS  TO  OPTIMIZE:");
		/*** Cost Function Cable I : Catenary constrain  ***/
		if(catenary_constraint==1){
			ROS_INFO(PRINTF_ORANGE"		- Optimize Catenary Obstacles Autodiff");
			for (int i = 0; i < statesLength.size(); ++i) {
				CostFunction* cost_function_cat_1  = new AutoDiffCostFunction<AutodiffCatenaryFunctor::CatenaryFunctor, 1, 4, 4, 2>
											(new AutodiffCatenaryFunctor::CatenaryFunctor(w_eta_1, w_eta_3, distance_catenary_obstacle, vec_len_cat_init[i], length_tether_max, nn_uav.kdtree, 
												nn_uav.obs_points, grid_3D, nn_trav.kdtree, nn_trav.obs_points, pose_reel_local.transform.translation, size_path, 
												vec_pose_init_uav[i], nh, mapFull_msg, step, write_data_residual, use_distance_function, user_name)); 
				problem.AddResidualBlock(cost_function_cat_1, NULL, statesPosUGV[i].parameter, statesPosUAV[i].parameter, statesLength[i].parameter);
				if (i == 0 || (fix_last_position_ugv && i == statesLength.size()-1))
					problem.SetParameterBlockConstant(statesLength[i].parameter);
				else{
					problem.SetParameterLowerBound(statesLength[i].parameter, 1, 0.1);
					problem.SetParameterUpperBound(statesLength[i].parameter, 1, length_tether_max);
				}
			}
		}
		if(catenary_constraint==0){
			ROS_INFO(PRINTF_ORANGE"		- Optimize Catenary Obstacles Numeric");
			for (int i = 0; i < statesLength.size(); ++i) {
				CostFunction* cost_function_cat_1  = new NumericDiffCostFunction<CatenaryFunctor, RIDDERS, 1, 4, 4, 2>
											(new CatenaryFunctor(w_eta_1, w_eta_3, distance_catenary_obstacle, vec_len_cat_init[i], length_tether_max, nn_uav.kdtree, 
												nn_uav.obs_points, grid_3D, nn_trav.kdtree, nn_trav.obs_points, pose_reel_local.transform.translation, size_path, 
												vec_pose_init_uav[i], nh, mapFull_msg, write_data_residual)); 
				problem.AddResidualBlock(cost_function_cat_1, NULL, statesPosUGV[i].parameter, statesPosUAV[i].parameter, statesLength[i].parameter);
				if (i == 0 || (fix_last_position_ugv && i == statesLength.size()-1))
					problem.SetParameterBlockConstant(statesLength[i].parameter);
				else{
					problem.SetParameterLowerBound(statesLength[i].parameter, 1, 0.1);
					problem.SetParameterUpperBound(statesLength[i].parameter, 1, length_tether_max);
				}
			}
		}
		if(catenary_length_constraint){
			ROS_INFO(PRINTF_ORANGE"		- Optimize Catenary Lentgh Autodiff");
			for (int i = 0; i < statesLength.size(); ++i) {
				CostFunction* cost_function_cat_2  = new AutoDiffCostFunction<CatenaryLengthFunctor::CatenaryLength, 1, 4, 4, 2>
											(new CatenaryLengthFunctor::CatenaryLength(w_eta_2, vec_len_cat_init[i], pose_reel_local.transform.translation, write_data_residual, user_name)); 
				problem.AddResidualBlock(cost_function_cat_2, NULL, statesPosUGV[i].parameter, statesPosUAV[i].parameter, statesLength[i].parameter);
				if (i == 0 || (fix_last_position_ugv && i == statesLength.size()-1))
					problem.SetParameterBlockConstant(statesLength[i].parameter);
				else{
					problem.SetParameterLowerBound(statesLength[i].parameter, 1, vec_len_cat_init[i]*0.73);
					problem.SetParameterUpperBound(statesLength[i].parameter, 1, length_tether_max);			
				}
			}		
		}
		/*** Cost Function Cable II : Dynamic Catenary constrain  ***/
		// for (int i = 0; i < statesLength.size() - 1; ++i) {
		// 	CostFunction* cost_function_cat_2  = new AutoDiffCostFunction<DynamicCatenaryFunctor, 1, 2, 2>(new DynamicCatenaryFunctor(w_lambda, dynamic_catenary)); 
		// 	problem.AddResidualBlock(cost_function_cat_2, NULL, statesLength[i].parameter, statesLength[i+1].parameter);
		// }
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

	std::cout << std::endl <<"Optimization Proccess Completed !!!" << std::endl << "===================================================" << std::endl << "Saving Temporal data in txt file ..." << std::endl;

	// Initializing Contraints for optimization	
	int n_coll_opt_traj_ugv_, n_coll_init_path_ugv_, n_coll_opt_traj_uav_, n_coll_init_path_uav_;
	int n_coll_opt_cat_;
	n_coll_opt_traj_ugv_ = n_coll_init_path_ugv_ = n_coll_opt_traj_uav_ = n_coll_init_path_uav_ = 0;

	finishigOptimizationAndGetDataResult(n_coll_opt_traj_ugv_, n_coll_init_path_ugv_, n_coll_opt_traj_uav_, n_coll_init_path_uav_, n_coll_opt_cat_);

	cleanVectors();		//Clear vector after optimization and previus the next iteration
	double time_sleep_ = 2.0;
	
	// Inform if is a feabible Trajectory or not, and visualize traj in RVIZ
	if (n_coll_opt_traj_ugv_ == 0 && n_coll_opt_traj_uav_ == 0 && n_coll_opt_cat_== 0 ){
		if (initial_cost==final_cost)
			ROS_INFO(PRINTF_ORANGE"\n		Optimizer Local Planner: Final Cost equal than Initial Cost");
		ROS_INFO(PRINTF_GREEN"\n\n\n		Optimizer Local Planner: Goal position successfully achieved through optimization trajectory\n\n\n");

		if (traj_in_rviz){
			if(pause_end_optimization){
				/********************* To obligate stop method and check Optimization result *********************/
				std::string y_ ;
				std::cout << " *** Press key to continue: " << std::endl;
				std::cin >> y_ ;
				/*************************************************************************************************/
			}
			publishOptimizedTraj();
			while(!finished_rviz_maneuver){
				ros::spinOnce();
				// printf("Waiting to finish maneuver in Rviz\n");
			};
			ros::Duration(time_sleep_).sleep();
			if(pause_end_optimization){
				/********************* To obligate stop method and check Optimization result *********************/
				std::string y_ ;
				std::cout << " *** Press key to continue: " << std::endl;
				std::cin >> y_ ;
				/*************************************************************************************************/
			}
			finished_rviz_maneuver = false;
		}
		else{
			ros::Duration(time_sleep_).sleep();
			if(pause_end_optimization){
				/********************* To obligate stop method and check Optimization result *********************/
				std::string y_ ;
				std::cout << " *** Press key to continue: " << std::endl;
				std::cin >> y_ ;
				/*************************************************************************************************/
			}
		}
		action_result.arrived = true;

		if (!get_path_from_file)
			execute_path_srv_ptr->setSucceeded(action_result);
	}
	else{
		if (initial_cost==final_cost)
				ROS_INFO(PRINTF_ORANGE"\n		Optimizer Local Planner: Final Cost equal than Initial Cost");
		ROS_INFO(PRINTF_RED"\n\n\n\n		Optimizer Local Planner: Goal position Not achieved through optimization trajectory\n\n\n");
		if (traj_in_rviz){
			if(pause_end_optimization){
				/********************* To obligate stop method and check Optimization result *********************/
				std::string y_ ;
				std::cout << " *** Press key to continue: " << std::endl;
				std::cin >> y_ ;
				/*************************************************************************************************/
			}
			publishOptimizedTraj();
			ros::Duration(time_sleep_).sleep();
			while(!finished_rviz_maneuver){
				ros::spinOnce();
				// printf("Waiting to finish maneuver in Rviz\n");
			};
			ros::Duration(time_sleep_).sleep();
			if(pause_end_optimization){
				/********************* To obligate stop method and check Optimization result *********************/
				std::string y_ ;
				std::cout << " *** Press key to continue: " << std::endl;
				std::cin >> y_ ;
				/*************************************************************************************************/
			}
		}
		else{
			ros::Duration(time_sleep_).sleep();
			if(pause_end_optimization){
				/********************* To obligate stop method and check Optimization result *********************/
				std::string y_ ;
				std::cout << " *** Press key to continue: " << std::endl;
				std::cin >> y_ ;
				/*************************************************************************************************/
			}
		}	
		finished_rviz_maneuver = false;
		action_result.arrived = false;

		if (!get_path_from_file)
			execute_path_srv_ptr->setSucceeded(action_result);
	}
	
	std::cout <<"Optimization Local Planner Proccess ended !!!!!!!!!!!!!!!!!!!!!!!" << std::endl << "==================================================="<< std::endl << std::endl << std::endl;
	resetFlags();
}

void OptimizerLocalPlanner::initializingParametersblock()
{
	//Set Parameter Blocks from path global information
	statesPosUGV.clear(); statesPosUAV.clear(); statesRotUGV.clear(); statesRotUAV.clear(); statesTime.clear(); statesLength.clear();
	
	printf("Initial States.Parameter[____]:  [  pos_x  pos_y  pos_z  rot_r  trot_p  rot_y pos_x  pos_y  pos_z  rot_x  rot_y  rot_z  rot_w  time_ugv  time_uav  length ]\n");
	for (size_t i = 0 ; i < vec_pose_init_uav.size(); i ++){
		parameterBlockPos parameter_block_pos_ugv;
		parameterBlockPos parameter_block_pos_uav;
		parameterBlockRot parameter_block_rot_ugv;
		parameterBlockRot parameter_block_rot_uav;
		parameterBlockTime parameter_block_time;
		parameterBlockLength parameter_block_length;
		
		/*** Parameters for UGV ***/
		// Position Parameters
		parameter_block_pos_ugv.parameter[0] = i; //id parameter
		parameter_block_pos_ugv.parameter[1] = vec_pose_init_ugv[i].x;
		parameter_block_pos_ugv.parameter[2] = vec_pose_init_ugv[i].y; 
		parameter_block_pos_ugv.parameter[3] = vec_pose_init_ugv[i].z; 
		// Rotation Parameters
		parameter_block_rot_ugv.parameter[0] = i; //id parameter 
		parameter_block_rot_ugv.parameter[1] = vec_init_rot_ugv[i].x;
		parameter_block_rot_ugv.parameter[2] = vec_init_rot_ugv[i].y; 
		parameter_block_rot_ugv.parameter[3] = vec_init_rot_ugv[i].z; 
		parameter_block_rot_ugv.parameter[4] = vec_init_rot_ugv[i].w;
		// Time Parameters
		parameter_block_time.parameter[0] = i; //id parameter 
		parameter_block_time.parameter[1] = vec_time_init[i];

		/*** Parameters for UAV ***/
		// Position Parameters
		parameter_block_pos_uav.parameter[0] = i; //id parameter
		parameter_block_pos_uav.parameter[1] = vec_pose_init_uav[i].x;
		parameter_block_pos_uav.parameter[2] = vec_pose_init_uav[i].y; 
		parameter_block_pos_uav.parameter[3] = vec_pose_init_uav[i].z; 
		//Rotation Parameters
		parameter_block_rot_uav.parameter[0] = i; //id parameter 
		parameter_block_rot_uav.parameter[1] = vec_init_rot_uav[i].x;
		parameter_block_rot_uav.parameter[2] = vec_init_rot_uav[i].y; 
		parameter_block_rot_uav.parameter[3] = vec_init_rot_uav[i].z; 
		parameter_block_rot_uav.parameter[4] = vec_init_rot_uav[i].w; 
			
		/*** Length Cable Parameters ***/
		parameter_block_length.parameter[0] = i; 
		parameter_block_length.parameter[1] = vec_len_cat_init[i];

		statesPosUGV.push_back(parameter_block_pos_ugv);
		statesPosUAV.push_back(parameter_block_pos_uav);
		statesRotUGV.push_back(parameter_block_rot_ugv);
		statesRotUAV.push_back(parameter_block_rot_uav);
		statesTime.push_back(parameter_block_time);
		statesLength.push_back(parameter_block_length);

		//Convert quaternion to euler angles
		double roll_ugv_init_, pitch_ugv_init_, yaw_ugv_init_;
		tf::Quaternion q_ugv_init_(parameter_block_rot_ugv.parameter[1],parameter_block_rot_ugv.parameter[2],parameter_block_rot_ugv.parameter[3],parameter_block_rot_ugv.parameter[4]);
		tf::Matrix3x3 M_ugv_init_(q_ugv_init_);	
		M_ugv_init_.getRPY(roll_ugv_init_, pitch_ugv_init_, yaw_ugv_init_);

		printf("Initial States.Parameter[%lu/%lu]: UGV=[%.3f %.3f %.3f / %.3f %.3f %.3f]",i,vec_pose_init_uav.size(),parameter_block_pos_ugv.parameter[1],
				parameter_block_pos_ugv.parameter[2],parameter_block_pos_ugv.parameter[3], roll_ugv_init_, pitch_ugv_init_, yaw_ugv_init_);
		printf(" UAV=[%.3f %.3f %.3f / %.3f %.3f %.3f %.3f] / [%.3f %.3f] / %.3f]\n", parameter_block_pos_uav.parameter[1],parameter_block_pos_uav.parameter[2],
				parameter_block_pos_uav.parameter[3], parameter_block_rot_uav.parameter[1],parameter_block_rot_uav.parameter[2],parameter_block_rot_uav.parameter[3],
				parameter_block_rot_uav.parameter[4], parameter_block_time.parameter[1],parameter_block_time.parameter[1],parameter_block_length.parameter[1]);
	}
}

void OptimizerLocalPlanner::finishigOptimizationAndGetDataResult(int &n_coll_opt_traj_ugv, int &n_coll_init_path_ugv, int &n_coll_opt_traj_uav, int &n_coll_init_path_uav, int &n_coll_opt_cat_)
{
	vec_pose_ugv_opt.clear();
	vec_pose_uav_opt.clear();
	vec_time_opt.clear();
	vec_len_cat_opt.clear();
	vec_opt_rot_ugv.clear();
	vec_opt_rot_uav.clear();
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
		double vLC = statesLength[i].parameter[1];
		vec_len_cat_opt.push_back(vLC);
		vec_time_opt.push_back(statesTime[i].parameter[1]);
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
	processingCatenary();
	exportOptimizedPath();
	
	bisectionCatenary bc;
	
	for(size_t i = 0; i < statesPosUAV.size(); i++){
		std::vector<geometry_msgs::Point> points_catenary_final;
		points_catenary_final.clear();

	  	// The Reel Position is consider above base_link_ugv
		geometry_msgs::Vector3 p_reel_ =  getReelPoint(statesPosUGV[i].parameter[1],statesPosUGV[i].parameter[2],statesPosUGV[i].parameter[3],statesRotUGV[i].parameter[1],statesRotUGV[i].parameter[2],statesRotUGV[i].parameter[3], statesRotUGV[i].parameter[4]);
		
		bc.configBisection(statesLength[i].parameter[1], p_reel_.x, p_reel_.y, p_reel_.z, 
							statesPosUAV[i].parameter[1], statesPosUAV[i].parameter[2], statesPosUAV[i].parameter[3]);
		bc.getPointCatenary3D(points_catenary_final, false);
		
		double _d_ = sqrt(pow(p_reel_.x - statesPosUAV[i].parameter[1],2) + 
						  pow(p_reel_.y - statesPosUAV[i].parameter[2],2) + 
						  pow(p_reel_.z - statesPosUAV[i].parameter[3],2));
		//Graph final catenary states
		mp_.markerPoints(catenary_marker, points_catenary_final, i, size_path, catenary_marker_pub_);	
		
		//Convert quaternion to euler angles
		double roll_ugv_, pitch_ugv_, yaw_ugv_;
		tf::Quaternion q_ugv_(statesRotUGV[i].parameter[1],statesRotUGV[i].parameter[2],statesRotUGV[i].parameter[3],statesRotUGV[i].parameter[4]);
		tf::Matrix3x3 M_(q_ugv_);	
		M_.getRPY(roll_ugv_, pitch_ugv_, yaw_ugv_);
		printf(PRINTF_REGULAR"Optimized States.Parameter[%lu/%lu]: UGV=[%.3f %.3f %.3f / %.3f %.3f %.3f %.3f] UAV=[%.3f %.3f %.3f / %.3f %.3f %.3f %.3f] , t=[%.3f/%.3f], l=[%.3f/points=%lu] d=[%.3f]\n", i,statesPosUAV.size(),
			   	statesPosUGV[i].parameter[1], statesPosUGV[i].parameter[2], statesPosUGV[i].parameter[3],
				statesRotUGV[i].parameter[1], statesRotUGV[i].parameter[2], statesRotUGV[i].parameter[3], statesRotUGV[i].parameter[4],
				statesPosUAV[i].parameter[1], statesPosUAV[i].parameter[2], statesPosUAV[i].parameter[3],
				statesRotUAV[i].parameter[1], statesRotUAV[i].parameter[2], statesRotUAV[i].parameter[3], statesRotUAV[i].parameter[4],
				statesTime[i].parameter[1],statesTime[i].parameter[1], statesLength[i].parameter[1],points_catenary_final.size(),_d_);
	}
	
	mp_.getMarkerPoints(points_ugv_marker, vec_pose_ugv_opt, "points_ugv_m",1);	// RED: colour_ = 0 ; GREEN : colour_ = 1 ; BLUE: colour_ = 2 ; YELLOW = 3 ; PURPLE = 4;
	mp_.getMarkerLines(lines_ugv_marker, vec_pose_ugv_opt, "lines_ugv_m",1);
	mp_.getMarkerPoints(points_uav_marker, vec_pose_uav_opt, "points_uav_m",2);
	mp_.getMarkerLines(lines_uav_marker, vec_pose_uav_opt, "lines_uav_m",2);
	traj_marker_ugv_pub_.publish(points_ugv_marker);
	traj_marker_ugv_pub_.publish(lines_ugv_marker);
	traj_marker_uav_pub_.publish(points_uav_marker);
	traj_marker_uav_pub_.publish(lines_uav_marker);

	if (write_data_for_analysis){
		getSmoothnessTrajectory(vec_pose_ugv_opt, vec_pose_uav_opt, v_opt_angles_smooth_ugv, v_opt_angles_smooth_uav);
		dm_.writeTemporalDataAfterOptimization(size_path, vec_pose_ugv_opt, vec_pose_uav_opt, vec_time_opt, vec_len_cat_opt, vec_opt_rot_ugv, 
											   vec_opt_rot_uav, v_opt_angles_smooth_ugv, v_opt_angles_smooth_uav);
		std::cout << "...Temporal data saved in txt file." << std::endl << std::endl;
		n_coll_opt_traj_ugv = n_coll_init_path_ugv = n_coll_opt_traj_uav = n_coll_init_path_uav = n_coll_opt_cat_ = 0;
		std::cout << "Saving data for analisys in txt file..." << std::endl;
		double opt_compute_time_ = (final_time_opt - start_time_opt).toSec(); //Compute Time Optimization
		dm_.getDataForOptimizerAnalysis(nn_ugv_obs.kdtree, nn_uav.kdtree, nn_ugv_obs.obs_points, nn_uav.obs_points, opt_compute_time_, "UGV", n_coll_init_path_ugv, n_coll_opt_traj_ugv);
		dm_.getDataForOptimizerAnalysis(nn_uav.kdtree, nn_uav.kdtree, nn_uav.obs_points, nn_uav.obs_points, opt_compute_time_, "UAV", n_coll_init_path_uav, n_coll_opt_traj_uav);
		// dm_.feasibilityAnalisysTrajectory(initial_cost, final_cost, successful_steps, unsuccessful_steps, time_optimazation, n_coll_opt_cat_);
		
		std::cout << "... data for analysis saved in txt file." << std::endl << std::endl;
	}
}

void OptimizerLocalPlanner::calculateDistanceVertices(vector<double> &_v_D_ugv,vector<double> &_v_D_uav)
{
	float x, y, z;
	_v_D_ugv.clear();
	_v_D_uav.clear();
    double pL = 0.0;
	double sum_distance_ = 0.0;

    for (size_t i = 0; i < vec_pose_init_ugv.size()-1; i++)
    {
    	x = vec_pose_init_ugv[i+1].x - vec_pose_init_ugv[i].x;
		y = vec_pose_init_ugv[i+1].y - vec_pose_init_ugv[i].y;
		z = vec_pose_init_ugv[i+1].z - vec_pose_init_ugv[i].z;
        pL= sqrt(x * x + y * y + z * z);
		sum_distance_ = sum_distance_ + pL;

		_v_D_ugv.push_back(pL);
    }
	double den_ = vec_pose_init_ugv.size()-(count_fix_points_initial_ugv );
	if (den_ <= 0.000001)
		initial_distance_states_ugv = 0.0;
	else
		initial_distance_states_ugv = sum_distance_ / den_; // Line when UGV initial states are considered in the same position

	pL = 0.0;
	sum_distance_ = 0.0;
	for (size_t i = 0; i < vec_pose_init_uav.size()-1; i++)
    {
    	x = vec_pose_init_uav[i+1].x - vec_pose_init_uav[i].x;
		y = vec_pose_init_uav[i+1].y - vec_pose_init_uav[i].y;
		z = vec_pose_init_uav[i+1].z - vec_pose_init_uav[i].z;
        pL= sqrt(x * x + y * y + z * z);
		
		sum_distance_ = sum_distance_ + pL;
		_v_D_uav.push_back(pL);
    }
	// initial_distance_states_uav = sum_distance_ / (vec_pose_init_uav.size()-1);
	int aux_n_p = round(vec_pose_init_uav.size()*0.1);
	initial_distance_states_uav = sum_distance_ / (vec_pose_init_uav.size()+aux_n_p);
}

void OptimizerLocalPlanner::getTemporalState(vector<double> &_time) 
{
	_time.clear();
	double sum_T = 0.0;
	double _dT = 0;
	int count_=0;
	min_T = 1000.0;
	for (size_t i= 0 ; i < vec_pose_init_uav.size(); i++){
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
	initial_time = sum_T/ (vec_pose_init_uav.size() - 1.0);
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

void OptimizerLocalPlanner::getSmoothnessTrajectory(vector<geometry_msgs::Vector3> v_pos2kin_ugv, vector<geometry_msgs::Vector3> v_pos2kin_uav, 
												   vector<double> &v_angles_kin_ugv, vector<double> &v_angles_kin_uav)
{
	v_angles_kin_ugv.clear();
	v_angles_kin_uav.clear();

	// Smoothness for ugv XY Axes
	for (size_t i=0 ; i < v_pos2kin_ugv.size()-2 ; i++){
		geometry_msgs::Point v1_ugv, v2_ugv;
		double dot_product_ugv, norm_vector1_ugv, norm_vector2_ugv, arg_norm_vector1_ugv, arg_norm_vector2_ugv, angle_ugv;
		v1_ugv.x=(v_pos2kin_ugv[i+1].x-v_pos2kin_ugv[i].x);
		v1_ugv.y=(v_pos2kin_ugv[i+1].y-v_pos2kin_ugv[i].y); 
		v1_ugv.z=(v_pos2kin_ugv[i+1].z-v_pos2kin_ugv[i].z);
		v2_ugv.x=(v_pos2kin_ugv[i+2].x-v_pos2kin_ugv[i+1].x);
		v2_ugv.y=(v_pos2kin_ugv[i+2].y-v_pos2kin_ugv[i+1].y); 
		v2_ugv.z=(v_pos2kin_ugv[i+2].z-v_pos2kin_ugv[i+1].z);
		dot_product_ugv = (v2_ugv.x * v1_ugv.x) + (v2_ugv.y * v1_ugv.y) + (v2_ugv.z * v2_ugv.z);
		arg_norm_vector1_ugv = (v1_ugv.x * v1_ugv.x) + (v1_ugv.y * v1_ugv.y) + (v2_ugv.z * v2_ugv.z);
		arg_norm_vector2_ugv = (v2_ugv.x * v2_ugv.x) + (v2_ugv.y * v2_ugv.y) + (v2_ugv.z * v2_ugv.z);

		if (arg_norm_vector1_ugv < 0.0001 || arg_norm_vector2_ugv < 0.0001 )
			angle_ugv = 0.0;
		else{
			norm_vector1_ugv = sqrt(arg_norm_vector1_ugv);
			norm_vector2_ugv = sqrt(arg_norm_vector2_ugv);
			double arg_ugv_ = (dot_product_ugv / (norm_vector1_ugv*norm_vector2_ugv)) ;
			if (arg_ugv_<= 1.000 && arg_ugv_ >= -1.000)
				angle_ugv = acos(dot_product_ugv / (norm_vector1_ugv*norm_vector2_ugv));
			else
				angle_ugv = 0.000;
		}
		v_angles_kin_ugv.push_back(angle_ugv);
	}

	for (size_t i=0 ; i < v_pos2kin_uav.size()-2 ; i++){
		geometry_msgs::Point v1_uav, v2_uav;
		double dot_product_uav, norm_vector1_uav, norm_vector2_uav, arg_norm_vector1_uav, arg_norm_vector2_uav, angle_uav;
		v1_uav.x=(v_pos2kin_uav[i+1].x-v_pos2kin_uav[i].x);
		v1_uav.y=(v_pos2kin_uav[i+1].y-v_pos2kin_uav[i].y); 
		v1_uav.z=(v_pos2kin_uav[i+1].z-v_pos2kin_uav[i].z);
		v2_uav.x=(v_pos2kin_uav[i+2].x-v_pos2kin_uav[i+1].x);
		v2_uav.y=(v_pos2kin_uav[i+2].y-v_pos2kin_uav[i+1].y); 
		v2_uav.z=(v_pos2kin_uav[i+2].z-v_pos2kin_uav[i+1].z);
		dot_product_uav = (v2_uav.x * v1_uav.x) + (v2_uav.y * v1_uav.y) + (v2_uav.z * v2_uav.z);
		arg_norm_vector1_uav = (v1_uav.x * v1_uav.x) + (v1_uav.y * v1_uav.y) + (v2_uav.z * v2_uav.z);
		arg_norm_vector2_uav = (v2_uav.x * v2_uav.x) + (v2_uav.y * v2_uav.y) + (v2_uav.z * v2_uav.z);
		
		if (arg_norm_vector1_uav < 0.0001 || arg_norm_vector2_uav < 0.0001 )
			angle_uav = 0.0;
		else{
			norm_vector1_uav = sqrt(arg_norm_vector1_uav);
			norm_vector2_uav = sqrt(arg_norm_vector2_uav);
			double arg_uav_ = (dot_product_uav / (norm_vector1_uav*norm_vector2_uav));
			if (arg_uav_<= 1.000 && arg_uav_ >= -1.000)
				angle_uav = acos(dot_product_uav / (norm_vector1_uav*norm_vector2_uav));
			else
				angle_uav = 0.000;
		}

		v_angles_kin_uav.push_back(angle_uav);
	}
}

void OptimizerLocalPlanner::processingCatenary()
{
	// Staff for fost-proccessing of catenary length
	parameterBlockLength parameter_block_post_length;
	statesPostLength.clear();
	// int count_cat_fail = 0;
	vec_len_cat_opt.clear();
	int first_ = 1;
	for(size_t i = 0; i < statesPosUAV.size(); i++){
		parameter_block_post_length.parameter[0] = i; //id parameter
		// Post-processing of catenary length
		geometry_msgs::Vector3 p_reel_ =  getReelPoint(statesPosUGV[i].parameter[1],statesPosUGV[i].parameter[2],statesPosUGV[i].parameter[3],
										  statesRotUGV[i].parameter[1],statesRotUGV[i].parameter[2],statesRotUGV[i].parameter[3], statesRotUGV[i].parameter[4]);
		double _d_ = sqrt(pow(p_reel_.x - statesPosUAV[i].parameter[1],2) + 
						  pow(p_reel_.y - statesPosUAV[i].parameter[2],2) + 
						  pow(p_reel_.z - statesPosUAV[i].parameter[3],2));

		if (_d_ > statesLength[i].parameter[1]){
			if(first_ == 1)
				printf(PRINTF_ORANGE"processingCatenary()\n");
			first_++;
			parameter_block_post_length.parameter[1] = _d_ * 1.005;	// 1.02 in catenary constraint
			printf(PRINTF_RED"UGV[%f %f %f] UAV[%f %f %f] reel[%f %f %f] Catenary number = %lu  [d=%f/L_o=%f/L_i=%f]\n",
			statesPosUGV[i].parameter[1],statesPosUGV[i].parameter[2],statesPosUGV[i].parameter[3], statesPosUAV[i].parameter[1],statesPosUAV[i].parameter[2],
			statesPosUAV[i].parameter[3],p_reel_.x, p_reel_.y, p_reel_.z,i,_d_,statesLength[i].parameter[1],vec_len_cat_init[i]);
		}
		else
			parameter_block_post_length.parameter[1] = statesLength[i].parameter[1];	
		
		statesPostLength.push_back(parameter_block_post_length);
		vec_len_cat_opt.push_back(parameter_block_post_length.parameter[1]);
	}
	statesLength.clear();
	statesLength = statesPostLength;
}

void OptimizerLocalPlanner::publishOptimizedTraj()
{
    ROS_INFO_COND(debug, "Publishing Trajectories");

	trajectory_msgs::MultiDOFJointTrajectoryPoint t_;
	trajectory_msgs::MultiDOFJointTrajectory trajectory_;
	std::vector<float> l_cat_;

    trajectory_.points.clear();

	t_.transforms.resize(2);
	t_.velocities.resize(2);
	t_.accelerations.resize(2);

	for (size_t i = 0; i <  statesPosUGV.size() ; i++){
		//For UGV
		t_.transforms[0].translation.x = statesPosUGV[i].parameter[1];
		t_.transforms[0].translation.y = statesPosUGV[i].parameter[2];
		t_.transforms[0].translation.z = statesPosUGV[i].parameter[3];
		t_.transforms[0].rotation.x = statesRotUGV[i].parameter[1];
		t_.transforms[0].rotation.y = statesRotUGV[i].parameter[2];
		t_.transforms[0].rotation.z = statesRotUGV[i].parameter[3];
		t_.transforms[0].rotation.w = statesRotUGV[i].parameter[4];
		if (i==0){
			t_.velocities[0].linear.x = 0.0;
			t_.velocities[0].linear.y = 0.0;
			t_.velocities[0].linear.z = 0.0;
		}else{ 
			t_.velocities[0].linear.x = (statesPosUGV[i].parameter[1] - statesPosUGV[i-1].parameter[1])/statesTime[i].parameter[1];
			t_.velocities[0].linear.y = (statesPosUGV[i].parameter[2] - statesPosUGV[i-1].parameter[2])/statesTime[i].parameter[1];
			t_.velocities[0].linear.z = (statesPosUGV[i].parameter[3] - statesPosUGV[i-1].parameter[3])/statesTime[i].parameter[1];
		}
		if (i==0 || i==statesPosUGV.size()-1){
			t_.accelerations[0].linear.x = 0.0;
			t_.accelerations[0].linear.y = 0.0;
			t_.accelerations[0].linear.z = 0.0;
		}
		else{
			t_.accelerations[0].linear.x= ((statesPosUGV[i+1].parameter[1] - statesPosUGV[i].parameter[1])/statesTime[i+1].parameter[1])-((statesPosUGV[i].parameter[1] - statesPosUGV[i-1].parameter[1])/statesTime[i].parameter[1]);
			t_.accelerations[0].linear.y= ((statesPosUGV[i+1].parameter[1] - statesPosUGV[i].parameter[1])/statesTime[i+1].parameter[1])-((statesPosUGV[i].parameter[1] - statesPosUGV[i-1].parameter[1])/statesTime[i].parameter[1]);
			t_.accelerations[0].linear.z= ((statesPosUGV[i+1].parameter[1] - statesPosUGV[i].parameter[1])/statesTime[i+1].parameter[1])-((statesPosUGV[i].parameter[1] - statesPosUGV[i-1].parameter[1])/statesTime[i].parameter[1]);
		}
		//For UAV
		t_.transforms[1].translation.x = statesPosUAV[i].parameter[1];
		t_.transforms[1].translation.y = statesPosUAV[i].parameter[2];
		t_.transforms[1].translation.z = statesPosUAV[i].parameter[3];
		t_.transforms[1].rotation.x = statesRotUAV[i].parameter[1];
		t_.transforms[1].rotation.y = statesRotUAV[i].parameter[2];
		t_.transforms[1].rotation.z = statesRotUAV[i].parameter[3];
		t_.transforms[1].rotation.w = statesRotUAV[i].parameter[4];
		if (i==0){
			t_.velocities[1].linear.x = 0.0;
			t_.velocities[1].linear.y = 0.0;
			t_.velocities[1].linear.z = 0.0;
		}else{  
			t_.velocities[1].linear.x = (statesPosUAV[i].parameter[1] - statesPosUAV[i-1].parameter[1])/statesTime[i].parameter[1];
			t_.velocities[1].linear.y = (statesPosUAV[i].parameter[2] - statesPosUAV[i-1].parameter[2])/statesTime[i].parameter[1];
			t_.velocities[1].linear.z = (statesPosUAV[i].parameter[3] - statesPosUAV[i-1].parameter[3])/statesTime[i].parameter[1];
		}
		if (i==0 || i==statesPosUGV.size()-1){
			t_.accelerations[1].linear.x = 0.0;
			t_.accelerations[1].linear.y = 0.0;
			t_.accelerations[1].linear.z = 0.0;
		}else{
			t_.accelerations[1].linear.x= ((statesPosUAV[i+1].parameter[1] - statesPosUAV[i].parameter[1])/statesTime[i+1].parameter[1])-((statesPosUAV[i].parameter[1] - statesPosUAV[i-1].parameter[1])/statesTime[i].parameter[1]);
			t_.accelerations[1].linear.y= ((statesPosUAV[i+1].parameter[1] - statesPosUAV[i].parameter[1])/statesTime[i+1].parameter[1])-((statesPosUAV[i].parameter[1] - statesPosUAV[i-1].parameter[1])/statesTime[i].parameter[1]);
			t_.accelerations[1].linear.z= ((statesPosUAV[i+1].parameter[1] - statesPosUAV[i].parameter[1])/statesTime[i+1].parameter[1])-((statesPosUAV[i].parameter[1] - statesPosUAV[i-1].parameter[1])/statesTime[i].parameter[1]);
		}
		// t_.time_from_start = statesTime[i].parameter[1];
		trajectory_.header.stamp = ros::Time::now();
		trajectory_.points.push_back(t_);
		l_cat_.push_back(statesLength[i].parameter[1]);
	}

	marsupial_optimizer::marsupial_trajectory_optimized msg;
	msg.trajectory = trajectory_;
	msg.length_catenary = l_cat_;
    trajectory_pub_.publish(msg);
}

void OptimizerLocalPlanner::cleanResidualConstraintsFile()
{
    std::string path_;

	path_ = path+files_residuals+"acceleration_uav.txt";
	if( remove(path_.c_str()) != 0 )
    	perror( "Error deleting file: acceleration_uav.txt" );
	// else
    // 	puts( "File successfully deleted: acceleration_uav.txt" );

	path_ = path+files_residuals+"acceleration_ugv.txt";
	if( remove(path_.c_str()) != 0 )
    	perror( "Error deleting file: acceleration_ugv.txt" );
	// else
    // 	puts( "File successfully deleted: acceleration_ugv.txt" );	

	path_ = path+files_residuals+"velocity_uav.txt";
	if( remove( path_.c_str()) != 0 )
    	perror( "Error deleting file: velocity_uav.txt" );

	path_ = path+files_residuals+"velocity_ugv.txt";
	if( remove( path_.c_str() ) != 0 )
    	perror( "Error deleting file: velocity_ugv.txt" );

	path_ = path+files_residuals+"equidistance_uav.txt";
	if( remove( path_.c_str() ) != 0 )
    	perror( "Error deleting file: equidistance_uav.txt" );

	path_ = path+files_residuals+"equidistance_ugv.txt";
	if( remove( path_.c_str() ) != 0 )
    	perror( "Error deleting file: equidistance_ugv.txt" );

	path_ = path+files_residuals+"smoothness_uav.txt";
	if( remove( path_.c_str() ) != 0 )
    	perror( "Error deleting file: smoothness_uav.txt" );

	path_ = path+files_residuals+"smoothness_ugv.txt";
	if( remove( path_.c_str() ) != 0 )
    	perror( "Error deleting file: smoothness_ugv.txt" );

	path_ = path+files_residuals+"obstacles_uav.txt";
	if( remove( path_.c_str() ) != 0 )
    	perror( "Error deleting file: obstacles_uav.txt");

	path_ = path+files_residuals+"obstacles_ugv.txt";
	if( remove( path_.c_str() ) != 0 )
    	perror( "Error deleting file: obstacles_ugv.txt");

	path_ = path+files_residuals+"traversability_ugv.txt";
	if( remove( path_.c_str() ) != 0 )
    	perror( "Error deleting file: traversability_ugv.txt");

	path_ = path+files_residuals+"time.txt";
	if( remove( path_.c_str() ) != 0 )
    	perror( "Error deleting file: time.txt");

	path_ = path+files_residuals+"catenary.txt";
	if( remove( path_.c_str() ) != 0 )
    	perror( "Error deleting file: catenary.txt");
	
	path_ = path+files_residuals+"catenary_length.txt";
	if( remove( path_.c_str() ) != 0 )
    	perror( "Error deleting file: catenary_length.txt");

	path_ = path+files_residuals+"catenary_length2.txt";
	if( remove( path_.c_str() ) != 0 )
    	perror( "Error deleting file: catenary_length2.txt");

	path_ = path+files_residuals+"catenary2.txt";
	if( remove( path_.c_str() ) != 0 )
    	perror( "Error deleting file: catenary2.txt");
}

void OptimizerLocalPlanner::exportOptimizedPath()
{
    time_t ttime = time(0);
    tm *local_time = localtime(&ttime);
    
    string year_, month_, day_, hour_, min_, sec_;
    year_ = to_string(1900 + local_time->tm_year);
    month_ = to_string(1 + local_time->tm_mon);
    day_ = to_string(local_time->tm_mday);
    hour_ = to_string(local_time->tm_hour); 
    min_ = to_string(local_time->tm_min); 
    sec_ = to_string(1 + local_time->tm_sec);

	double d_to_interp_, d1_, d2_, cat_inter_;
	int r_;
	bool interpol_;
	vector<geometry_msgs::Vector3> v_interp_pose_ugv, v_interp_pose_uav;
	vector<geometry_msgs::Quaternion> v_interp_rot_ugv, v_interp_rot_uav;
	vector<float> vec_interp_len_cat;
	geometry_msgs::Vector3 p_int_ugv_, p_int_uav_;

	// Interpolate vector
	printf("vec_pose_ugv_opt.size()=%lu , vec_pose_uav_opt.size()=%lu , vec_len_cat_opt.size()=%lu\n",vec_pose_ugv_opt.size(),vec_pose_uav_opt.size(),vec_len_cat_opt.size());
	for (int i=0 ; i < vec_pose_ugv_opt.size()-1; i++){
		d_to_interp_ = 0.2;
		interpol_ = false;
		d1_ = sqrt ( pow(vec_pose_ugv_opt[i+1].x - vec_pose_ugv_opt[i].x,2) + pow(vec_pose_ugv_opt[i+1].y - vec_pose_ugv_opt[i].y,2) + pow(vec_pose_ugv_opt[i+1].z - vec_pose_ugv_opt[i].z,2) );
		d2_ = sqrt ( pow(vec_pose_uav_opt[i+1].x - vec_pose_uav_opt[i].x,2) + pow(vec_pose_uav_opt[i+1].y - vec_pose_uav_opt[i].y,2) + pow(vec_pose_uav_opt[i+1].z - vec_pose_uav_opt[i].z,2) );
		if (d1_ > d_to_interp_ ){
			if (d1_ >= d2_)
				r_ = floor(d1_/d_to_interp_);
			interpol_ = true;
		}
		if (d2_ > d_to_interp_ ){
			if (d2_ > d1_)
				r_ = floor(d2_/d_to_interp_);
			interpol_ = true;
		}
		if (interpol_){
			for (int j=0 ; j < r_ ; j++){
				p_int_ugv_.x = vec_pose_ugv_opt[i].x + (j+1)*(vec_pose_ugv_opt[i+1].x - vec_pose_ugv_opt[i].x)/(r_+1);
				p_int_ugv_.y = vec_pose_ugv_opt[i].y + (j+1)*(vec_pose_ugv_opt[i+1].y - vec_pose_ugv_opt[i].y)/(r_+1);
				p_int_ugv_.z = vec_pose_ugv_opt[i].z + (j+1)*(vec_pose_ugv_opt[i+1].z - vec_pose_ugv_opt[i].z)/(r_+1);
				p_int_uav_.x = vec_pose_uav_opt[i].x + (j+1)*(vec_pose_uav_opt[i+1].x - vec_pose_uav_opt[i].x)/(r_+1);
				p_int_uav_.y = vec_pose_uav_opt[i].y + (j+1)*(vec_pose_uav_opt[i+1].y - vec_pose_uav_opt[i].y)/(r_+1);
				p_int_uav_.z = vec_pose_uav_opt[i].z + (j+1)*(vec_pose_uav_opt[i+1].z - vec_pose_uav_opt[i].z)/(r_+1);
				cat_inter_ = vec_len_cat_opt[i] + (j+1)*(vec_len_cat_opt[i+1] - vec_len_cat_opt[i])/(r_+1);
				v_interp_pose_ugv.push_back(p_int_ugv_);
				v_interp_rot_ugv.push_back(vec_opt_rot_ugv[i]);
				v_interp_pose_uav.push_back(p_int_uav_);
				v_interp_rot_uav.push_back(vec_opt_rot_uav[i]);
				vec_interp_len_cat.push_back(cat_inter_);
			}
		}else{
			v_interp_pose_ugv.push_back(vec_pose_ugv_opt[i]);
			v_interp_rot_ugv.push_back(vec_opt_rot_ugv[i]);
			v_interp_pose_uav.push_back(vec_pose_uav_opt[i]);
			v_interp_rot_uav.push_back(vec_opt_rot_uav[i]);
			vec_interp_len_cat.push_back(vec_len_cat_opt[i]);
		}
	}
	vec_pose_ugv_opt.clear();
	vec_opt_rot_ugv.clear();
	vec_pose_uav_opt.clear();
	vec_opt_rot_uav.clear();
	vec_len_cat_opt.clear();
	vec_pose_ugv_opt = v_interp_pose_ugv;
	vec_opt_rot_ugv = v_interp_rot_ugv;
	vec_pose_uav_opt = v_interp_pose_uav;
	vec_opt_rot_uav = v_interp_rot_uav;
	vec_len_cat_opt = vec_interp_len_cat;
	printf("vec_pose_ugv_opt.size()=%lu , vec_pose_uav_opt.size()=%lu vec_interp_len_cat=%lu\n",
	v_interp_pose_ugv.size(),v_interp_pose_ugv.size(),vec_interp_len_cat.size());

    // Root of our file
    YAML::Node root_ugv, root_uav, root_tether;

    // Populate emitter
    YAML::Emitter emitter;

    // // Create a node listing some values
    root_ugv["marsupial_ugv"] = YAML::Node(YAML::NodeType::Map);
    // // We now will write our values under root["MyNode"]
    YAML::Node node = root_ugv["marsupial_ugv"];
    // YAML::Node node;
    // Write some values
    int size_ = vec_pose_ugv_opt.size();

    node["header"] = "marsupial_ugv";
    node["seq"] = 1;
    node["stamp"] = hour_+min_+sec_;
    node["frame_id"] = "ugv";
    node["size"] = size_;
    
    for(int i=0 ; i < vec_pose_ugv_opt.size(); i++){
		node["poses"+to_string(i)]["header"] = "ugv"+to_string(i);
        node["poses"+to_string(i)]["seq"] = i;
        node["poses"+to_string(i)]["frame_id"] = "ugv";  
        node["poses"+to_string(i)]["pose"]["position"]["x"] = vec_pose_ugv_opt[i].x;
        node["poses"+to_string(i)]["pose"]["position"]["y"] = vec_pose_ugv_opt[i].y;
        node["poses"+to_string(i)]["pose"]["position"]["z"] = vec_pose_ugv_opt[i].z;
        node["poses"+to_string(i)]["pose"]["orientation"]["x"] = vec_opt_rot_ugv[i].x;
        node["poses"+to_string(i)]["pose"]["orientation"]["y"] = vec_opt_rot_ugv[i].y;
        node["poses"+to_string(i)]["pose"]["orientation"]["z"] = vec_opt_rot_ugv[i].z;
        node["poses"+to_string(i)]["pose"]["orientation"]["w"] = vec_opt_rot_ugv[i].w;
    }

    emitter << root_ugv;

    // // Create a node listing some values
    root_uav["marsupial_uav"] = YAML::Node(YAML::NodeType::Map);
    // // We now will write our values under root["MyNode"]
    node = root_uav["marsupial_uav"];
	size_ = vec_pose_uav_opt.size();
    // YAML::Node node;
    // Write some values
    node["header"] = "marsupial_uav";
    node["seq"] = 1;
    node["stamp"] = hour_+min_+sec_ ;
    node["frame_id"] = "base_link_uav";
    node["size"] = size_;
    for(int i=0 ; i < vec_pose_uav_opt.size(); i++){
        node["poses"+to_string(i)]["header"] = "uav"+to_string(i);
        node["poses"+to_string(i)]["seq"] = i;
        node["poses"+to_string(i)]["frame_id"] = "uav";  
        node["poses"+to_string(i)]["pose"]["position"]["x"] = vec_pose_uav_opt[i].x;
        node["poses"+to_string(i)]["pose"]["position"]["y"] = vec_pose_uav_opt[i].y;
        node["poses"+to_string(i)]["pose"]["position"]["z"] = vec_pose_uav_opt[i].z;
        node["poses"+to_string(i)]["pose"]["orientation"]["x"] = vec_opt_rot_uav[i].x;
        node["poses"+to_string(i)]["pose"]["orientation"]["y"] = vec_opt_rot_uav[i].y;
        node["poses"+to_string(i)]["pose"]["orientation"]["z"] = vec_opt_rot_uav[i].z;
        node["poses"+to_string(i)]["pose"]["orientation"]["w"] = vec_opt_rot_uav[i].w;
    }

    // Populate emitter
    emitter << root_uav;

	    // // Create a node listing some values
    root_tether["tether"] = YAML::Node(YAML::NodeType::Map);
    // // We now will write our values under root["MyNode"]
    node = root_tether["tether"];
	size_ = vec_len_cat_opt.size();
    // YAML::Node node;
    // Write some values
    node["header"] = "tether";
    node["seq"] = 1;
    node["stamp"] = hour_+min_+sec_ ;
    node["frame_id"] = "frame_tether";
    node["size"] = size_;
    for(int i=0 ; i < vec_len_cat_opt.size(); i++){
        node["length"+to_string(i)]["header"] = "tether"+to_string(i);
        node["length"+to_string(i)]["seq"] = i;
        node["length"+to_string(i)]["frame_id"] = "tether_length";  
        node["length"+to_string(i)]["length"] = vec_len_cat_opt[i];
    }

    // Populate emitter
    emitter << root_tether;

    // Write to file
    std::ofstream fout;
	fout.open(path_mission_file+"optimized_path_"+year_+"_"+month_+"_"+day_+"_"+hour_+min_+sec_ +".yaml", std::ofstream::app);
    fout << emitter.c_str();

	std::cout << "Saved Path Optimized" << std::endl << std::endl;
}

void OptimizerLocalPlanner::getParableParameter(vector<geometry_msgs::Vector3> v_p_init_ugv_, vector<geometry_msgs::Vector3> v_p_init_uav_, vector<float> &v_l_cat_init_){

    GetParableParameter GPP(v_p_init_ugv_, v_p_init_uav_, v_l_cat_init_, vec_init_rot_ugv, pose_reel_local);
	GPP.ParametersParable();
	v_l_cat_init_.clear();                    //Check This two lines, should be fixed in a previous method the length
	v_l_cat_init_ = GPP.vec_len_cat_init;     //Check This two lines, should be fixed in a previous method the length

}