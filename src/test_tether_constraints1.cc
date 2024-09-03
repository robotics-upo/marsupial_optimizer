/*
Optimizer Local Planner based in Ceres Solver for Marsupial Robotics COnfiguration 
Simon Martinez-Rozas, 2023
Service Robotics Lab, University Pablo de Olavide , Seville, Spain 
 */

#include "marsupial_optimizer/test_tether_constraints.h"

TestTetherConstraints::TestTetherConstraints(std::string node_name_)
{
	
	nh.reset(new ros::NodeHandle("~"));

    tfBuffer.reset(new tf2_ros::Buffer);
    tf2_list.reset(new tf2_ros::TransformListener(*tfBuffer));
    tf_list_ptr.reset(new tf::TransformListener(ros::Duration(5)));

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

	nh->param<bool>("use_loss_function",use_loss_function, false);
	nh->param<bool>("use_catenary_as_tether",use_catenary_as_tether, true);

	nh->param<bool>("tether_obstacle_constraint",tether_obstacle_constraint, false);
	nh->param<bool>("tether_length_constraint",tether_length_constraint, false);
	nh->param<bool>("tether_parameters_constraint",tether_parameters_constraint, false);
	nh->param<bool>("print_point_in_graph_marker",print_point_in_graph_marker, false);

	nh->param<double>("w_eta_1", w_eta_1,0.1);
	nh->param<double>("w_eta_2", w_eta_2,0.1);
	nh->param<double>("w_eta_3", w_eta_3,0.1);

	nh->param<int>("n_iter_opt", n_iter_opt,200);
	nh->param<double>("distance_tether_obstacle", distance_tether_obstacle, 0.1);
	nh->param<double>("length_tether_max", length_tether_max,20.0);

	nh->param<bool>("write_data_residual",write_data_residual, false);
	nh->param("path", path, (std::string) "~/");
	nh->param("path_mission_file", path_mission_file, (std::string) "~/missions/cfg/");
	nh->param("pc_user_name", user_name, (std::string) "simon");
	nh->param("map_path", map_path, (std::string) "map.bt");
	nh->param("map_path_trav", map_path_trav, (std::string) "map_trav.bt");
	nh->param("map_path_obst", map_path_obst, (std::string) "map_obst.bt");

	nh->param<bool>("debug", debug, true);
 	nh->param<bool>("show_config", showConfig, true);
 	nh->param<bool>("use_distance_function", use_distance_function, true);

	ROS_INFO_COND(showConfig, PRINTF_BLUE "TESTING CONSTRAINT FOR TETHER 3D Node Configuration:\n");
		
	step = map_resolution;
	step_inv = 1.0 / step;

	initializeSubscribers();
	initializePublishers();
	setupOptimizer();
	ROS_INFO(PRINTF_BLUE"Optimizer_Local_Planner: use_distance_function: %s",use_distance_function?"true":"false");
	printf("Optimizer_Local_Planner: alpha_ugv=[%f] beta_ugv=[%f] gamma_ugv=[%f] kappa_ugv=[%f] theta_ugv=[%f]\n"
							"\t\t\t alpha_uav=[%f] beta_uav=[%f] gamma_uav=[%f] kappa_uav=[%f] epsilon_uav=[%f] zeta_uav=[%f]\n"
							"\t\t\t delta_t=[%f] eta_tether(obt len par)=[%f %f %f]",
							w_alpha_ugv, w_beta_ugv, w_gamma_ugv, w_kappa_ugv, w_theta_ugv, 
							w_alpha_uav, w_beta_uav, w_gamma_uav, w_kappa_uav, w_epsilon_uav, w_zeta_uav,
						  	w_delta, w_eta_1, w_eta_2, w_eta_3);
	
	std::string _node_name = "grid3D_optimizer_node";
	std::string _node_name_trav = "grid3D_optimizer_trav_node";
	std::string _node_name_obst = "grid3D_optimizer_obst_node";
	grid_3D = new Grid3d(_node_name, map_path);
    ROS_INFO_COND(true, PRINTF_BLUE "Initialazing Trilinear Interpolation (grid3D) in Optimizer");
	grid_3D->computeTrilinearInterpolation();
	// grid_3D_trav = new Grid3d(_node_name_trav, map_path_trav);
    // ROS_INFO_COND(true, PRINTF_BLUE "Initialazing Trilinear Interpolation (grid3D_trav) in Optimizer");
	// grid_3D_trav->computeTrilinearInterpolation();
	// grid_3D_obst = new Grid3d(_node_name_obst, map_path_obst);
    // ROS_INFO_COND(true, PRINTF_BLUE "Initialazing Trilinear Interpolation (grid3D_trav) in Optimizer");
	// grid_3D_obst->computeTrilinearInterpolation();
    // ROS_INFO_COND(true, PRINTF_BLUE "Finished Trilinear Interpolation (grid3D) in Optimizer");

	CheckCM = new CatenaryCheckerManager(node_name_);
	bool use_parabola_ = true;
	bool just_line_of_sigth_ = false;
	CheckCM->Init(grid_3D, distance_tether_obstacle, distance_obstacle_ugv, distance_obstacle_uav, length_tether_max, ws_z_min, step, 
	use_parabola_, use_distance_function, pose_reel_local.transform.translation, just_line_of_sigth_, use_catenary_as_tether);

	ros::Duration(2.0).sleep();
    getReelPose(); // To get init pos reel for optimization process
	// if (mapReceivedFull && mapReceivedTrav)
		executeOptimizerPathGoalCB();
}

void TestTetherConstraints::initializeSubscribers()
{
	octomap_ws_sub_ = nh->subscribe( "/octomap_point_cloud_centers", 1,  &TestTetherConstraints::readOctomapCallback, this);
    local_map_sub = nh->subscribe<octomap_msgs::Octomap>("/octomap_binary_local", 1, &TestTetherConstraints::collisionMapCallBack, this);
    local_trav_map_sub = nh->subscribe<octomap_msgs::Octomap>("/oct_trav/octomap_binary", 1, &TestTetherConstraints::traversableMapCallBack, this);
    point_cloud_ugv_obstacles_sub_ = nh->subscribe( "/region_growing_obstacles_pc_map", 1,  &TestTetherConstraints::readPointCloudObstaclesUGVCallback, this);
    point_cloud_ugv_traversability_sub_ = nh->subscribe( "/region_growing_traversability_pc_map", 1,  &TestTetherConstraints::readPointCloudTraversabilityUGVCallback, this);

	ROS_INFO(PRINTF_BLUE"Optimizer_Local_Planner: Subscribers Initialized");
}

void TestTetherConstraints::initializePublishers()
{
	tether_marker_init_pub_ = nh->advertise<visualization_msgs::MarkerArray>("init_tether_marker", 200);
	tether_marker_opt_pub_ = nh->advertise<visualization_msgs::MarkerArray>("opt_tether_marker", 200);

  	traj_marker_ugv_pub_ = nh->advertise<visualization_msgs::MarkerArray>("init_trajectory_ugv_marker", 2);
  	traj_marker_uav_pub_ = nh->advertise<visualization_msgs::MarkerArray>("init_trajectory_uav_marker", 2);
	traj_opt_marker_ugv_pub_ = nh->advertise<visualization_msgs::MarkerArray>("opt_trajectory_uav_marker", 2);
  	traj_opt_marker_uav_pub_ = nh->advertise<visualization_msgs::MarkerArray>("opt_trajectory_uav_marker", 2);

  	ROS_INFO(PRINTF_BLUE"Optimizer_Local_Planner: Publishers Initialized");
}

void TestTetherConstraints::resetFlags()
{
  	mapReceivedFull = false;
  	mapReceivedTrav = false;
}

void TestTetherConstraints::cleanVectors()
{
	vec_pose_ugv_init.clear();	vec_pose_uav_init.clear();  v_tether_params_init.clear();
	vec_len_tether_init.clear(); vec_rot_ugv_init.clear(); // Boot are not in use
	statesPosUGV.clear(); 	statesPosUAV.clear();	statesTetherParams.clear();
}

void TestTetherConstraints::setupOptimizer()
{
	options.max_num_iterations = n_iter_opt;
	options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
	options.minimizer_progress_to_stdout = true;
	options.num_threads = 1;
}

void TestTetherConstraints::readOctomapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	ROS_INFO(PRINTF_MAGENTA "KDTree : Preparing KDTree UAV");
	nn_uav.setInput(*msg);
  	ROS_INFO_COND(debug, PRINTF_BLUE "Local Planner: Received Point Cloud for KDTree UAV");
}

void TestTetherConstraints::readPointCloudTraversabilityUGVCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	ROS_INFO(PRINTF_MAGENTA "KDTree : Preparing KDTree UGV Traversability");
	nn_trav.setInput(*msg);
  	ROS_INFO_COND(debug, PRINTF_BLUE "Local Planner: Received Point Cloud for KDTree traversability UGV");
}

void TestTetherConstraints::readPointCloudObstaclesUGVCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	ROS_INFO(PRINTF_MAGENTA "KDTree : Preparing KDTree UGV Obstacles");
    pc_obs_ugv = msg;
	nn_ugv_obs.setInput(*msg);
  	ROS_INFO_COND(debug, PRINTF_BLUE "Local Planner: Received Point Cloud for KDTree obstacles UGV");
}

void TestTetherConstraints::collisionMapCallBack(const octomap_msgs::OctomapConstPtr &msg)
{
	mapReceivedFull = true;
	mapFull_msg = (octomap::OcTree *)octomap_msgs::binaryMsgToMap(*msg);
}

void TestTetherConstraints::traversableMapCallBack(const octomap_msgs::OctomapConstPtr &msg)
{
    mapReceivedTrav = true;
	mapTrav_msg = (octomap::OcTree *)octomap_msgs::binaryMsgToMap(*msg);
}

void TestTetherConstraints::executeOptimizerPathGoalCB()
{
  	ROS_INFO(PRINTF_GREEN "Optimizer Local Planner : Path received in action server mode\n");

	int row = 10;
	int colum = 9;
	size_path = row;

    double data[row][colum] = {
//  {8.50000,  1.80000, 0.00000,  8.50000, 1.50000 , 1.00000, 4.185,  0.811, 0.380},
//  {8.50000,  1.80000, 0.00000,  8.15000, 1.40000 , 1.20000, 1.536,  0.726, 0.380},
//  {8.50000,  1.80000, 0.00000,  8.05000, 1.05000 , 1.40000, 0.657,  0.592, 0.380},
//  {8.50000,  1.80000, 0.00000,  7.75000, 0.25000 , 1.45000, 0.197,  0.281, 0.380},
//  {8.20000,  1.20000, 0.00000,  7.15000, -0.35000, 1.40000, 0.170,  0.227, 0.380},
//  {8.10000,  1.08333, 0.00000,  6.75000, -0.40000, 1.45000, 0.231,  0.071, 0.380},
//  {8.00000,  0.96667, 0.00000,  6.45000, -1.00000, 1.50000, 0.197, -0.046, 0.380},
//  {7.90000,  0.85000, 0.00000,  6.10000, -1.20000, 1.40000, 0.103,  0.094, 0.380},
//  {7.90000,  0.45000, 0.00000,  5.70000, -1.20000, 1.40000, 0.102,  0.091, 0.380},
//  {7.85313,  0.40938, 0.00000,  5.40000, -0.90000, 1.40000, 0.112,  0.055, 0.380},
//  {7.80625,  0.36875, 0.00000,  5.05000, -1.10000, 1.40000, 0.105, -0.000, 0.380},
//  {7.75938,  0.32813, 0.00000,  4.65000, -0.90000, 1.40000, 0.101, -0.034, 0.380},
//  {7.71250,  0.28750, 0.00000,  4.25000, -0.75000, 1.40000, 0.095, -0.062, 0.380},
//  {7.66563,  0.24688, 0.00000,  3.90000, -0.50000, 1.40000, 0.014,  0.211, 0.380},
//  {7.61875,  0.20625, 0.00000,  3.60000, -0.75000, 1.40000, 0.014,  0.190, 0.380},
//  {7.57188,  0.16563, 0.00000,  3.25000, -0.95000, 1.35000, 0.012,  0.163, 0.380},
//  {7.52500,  0.12500, 0.00000,  3.00000, -1.25000, 1.35000, 0.012,  0.149, 0.380},
//  {7.47813,  0.08438, 0.00000,  2.60000, -1.20000, 1.40000, 0.011,  0.145, 0.380},
//  {7.43125,  0.04375, 0.00000,  2.20000, -1.15000, 1.35000, 0.010,  0.128, 0.380},
//  {7.38438,  0.00313, 0.00000,  1.85000, -1.00000, 1.55000, 0.011,  0.146, 0.380},
//  {7.33750, -0.03750, 0.00000,  1.50000, -0.85000, 1.70000, 0.011,  0.156, 0.380},
//  {7.29063, -0.07812, 0.00000,  1.10000, -0.95000, 1.70000, 0.010,  0.146, 0.380},
//  {7.24375, -0.11875, 0.00000,  1.05000, -0.90000, 2.05000, 0.064, -0.132, 0.380},
//  {7.19688, -0.15938, 0.00000,  0.65000, -0.95000, 2.00000, 0.012,  0.165, 0.380},
//  {7.15000, -0.20000, 0.00000, -0.05000, -0.85000, 2.70000, 0.038,  0.049, 0.380},
//  {6.80000,  0.00000, 0.00000, -0.35000, -0.80000, 2.90000, 0.038,  0.074, 0.380},
//  {6.66667, -0.03333, 0.00000, -0.20000, -0.75000, 3.25000, 0.056,  0.027, 0.380},
//  {6.53333, -0.06667, 0.00000, -0.25000, -0.45000, 3.50000, 0.082, -0.095, 0.380},
//  {6.40000, -0.10000, 0.00000, -0.10000, -0.45000, 3.85000, 0.091, -0.060, 0.380},
//  {6.30000, -0.13750, 0.00000,  0.10000, -0.10000, 3.75000, 0.110, -0.138, 0.380},
//  {6.20000, -0.17500, 0.00000,  0.20000, -0.10000, 4.10000, 0.162, -0.354, 0.380},
//  {6.10000, -0.21250, 0.00000,  0.40000, -0.35000, 4.25000, 0.193, -0.424, 0.380},
 {6.00000, -0.25000, 0.00000,  0.65000, -0.05000, 4.35000, 0.230, -0.491, 0.380},
 {5.60000, -0.35000, 0.00000,  0.70000, -0.10000, 4.70000, 0.292, -0.553, 0.381},
 {5.20000, -0.50000, 0.00000,  0.65000, -0.05000, 4.80000, 0.335, -0.564, 0.380},
 {4.80000, -0.60000, 0.00000,  0.60000,  0.00000, 5.60000, 0.446, -0.663, 0.380},
 {4.66667, -0.56667, 0.00000,  0.50000, -0.30000, 5.80000, 0.470, -0.665, 0.380},
 {4.53333, -0.53333, 0.00000,  0.65000, -0.35000, 6.15000, 0.581, -0.776, 0.380},
 {4.40000, -0.50000, 0.00000,  0.90000, -0.55000, 6.30000, 0.733, -0.876, 0.381},
 {4.26667, -0.50000, 0.00000,  1.10000, -0.50000, 6.70000, 0.714, -0.266, 0.381},
 {4.13333, -0.50000, 0.00000,  1.05000, -0.50000, 6.85000, 0.931, -0.773, 0.383},
 {4.00000, -0.50000, 0.00000,  1.00000, -0.50000, 7.00000, 1.185, -1.348, 0.380}
 };
	
	parameterBlockPos parameter_block_pos_ugv;
	parameterBlockPos parameter_block_pos_uav;
	parameterBlockTether parameter_block_tether_params;
	geometry_msgs::Point pos_ugv_, pos_uav_;
	tether_parameters param_value_;

	cleanVectors();

	for (int i = 0; i < row; i++) {
        // for (int j = 0; j < colum; j++) {
			// UGV position
			parameter_block_pos_ugv.parameter[0] = i;
			parameter_block_pos_ugv.parameter[1] = data[i][0];
			parameter_block_pos_ugv.parameter[2] = data[i][1]; 
			parameter_block_pos_ugv.parameter[3] = data[i][2];
			pos_ugv_.x = data[i][0];
			pos_ugv_.y = data[i][1];
			pos_ugv_.z = data[i][2]; 
			// UAV position
			parameter_block_pos_uav.parameter[0] = i;
			parameter_block_pos_uav.parameter[1] = data[i][3];
			parameter_block_pos_uav.parameter[2] = data[i][4]; 
			parameter_block_pos_uav.parameter[3] = data[i][5]; 
			pos_uav_.x = data[i][3];
			pos_uav_.y = data[i][4];
			pos_uav_.z = data[i][5]; 
			// Parameters tether
			parameter_block_tether_params.parameter[0] = i;
			parameter_block_tether_params.parameter[1] = data[i][6];
			parameter_block_tether_params.parameter[2] = data[i][7]; 
			parameter_block_tether_params.parameter[3] = data[i][8];
			param_value_.a = data[i][6];
			param_value_.b = data[i][7];
			param_value_.c = data[i][8];  

			vec_pose_ugv_init.push_back(pos_ugv_);
			vec_pose_uav_init.push_back(pos_uav_);
			v_tether_params_init.push_back(param_value_);
			statesPosUGV.push_back(parameter_block_pos_ugv);
			statesPosUAV.push_back(parameter_block_pos_uav);
			statesTetherParams.push_back(parameter_block_tether_params);
        // }
    }
	ROS_INFO("Optimizer Local Planner : Initial condition of the solution");
	CheckStatusTetherCollision(vec_pose_ugv_init, vec_pose_uav_init, v_tether_params_init);

	for (size_t i=0; i< row; i++){
		printf("\toptimizer_local_planer[%lu/%i] :  ugv=[%.5f %.5f %.5f]  uav=[%.5f %.5f %.5f]  params_cat=[%.3f %.3f %.3f]\n", 
        i, row,
		data[i][0], data[i][1], data[i][2], 
		data[i][3], data[i][4], data[i][5], 
		data[i][6], data[i][7], data[i][8]);
	}

std::cout << "Pose Reel: "<< pose_reel_local.transform.translation.z << std::endl;
	ROS_INFO(PRINTF_GREEN "Optimizer Local Planner : Graph Tether initial solution");
	graphTetherAndPathMarker(vec_pose_ugv_init, vec_pose_uav_init, vec_rot_ugv_init, v_tether_params_init, vec_len_tether_init, 5, 6, 2,
							  traj_marker_ugv_pub_, traj_marker_uav_pub_, tether_marker_init_pub_, tether_marker_init);

	/********************* To obligate pause method and check Planning result *********************/
        std::string y_ ;
        std::cout << " *** Before Optimization proccess" ;
        std::cout << " : Press key to continue : " ;
        std::cin >> y_ ;
    /*************************************************************************************************/

  	// Initialize optimizer
  	Problem problem;
	LossFunction* loss_function = NULL;
	if(use_loss_function){
		loss_function = new CauchyLoss(0.5);
	}

	// Initializing Contraints for optimization	
	/****************************   Tether Constraints  ****************************/	
		ROS_INFO(PRINTF_ORANGE" PREPARING  CATENARY  TETHER  BLOCKS  TO  OPTIMIZE:");
		/*** Cost Function Cable I : Tether constrain  ***/
			if(use_catenary_as_tether){ 
				if(tether_obstacle_constraint){
					ROS_INFO(PRINTF_ORANGE"		- Optimize Tether Obstacles Autodiff - Catenary");
					for (int i = 0; i < statesTetherParams.size(); ++i) {
						CostFunction* cost_function_par_1  = new AutoDiffCostFunction<AutodiffTetherObstacleFunctor::TetherObstacleFunctor, 1, 4, 4, 4> //Residual, ugvPos, uavPos, tetherParams
									(new AutodiffTetherObstacleFunctor::TetherObstacleFunctor(w_eta_1, grid_3D, pose_reel_local.transform.translation, distance_tether_obstacle, write_data_residual, user_name)); 
							problem.AddResidualBlock(cost_function_par_1, loss_function, statesPosUGV[i].parameter, statesPosUAV[i].parameter, statesTetherParams[i].parameter);
							problem.SetParameterLowerBound(statesTetherParams[i].parameter, 2, 0.01);
							problem.SetParameterLowerBound(statesTetherParams[i].parameter, 3, 0.1);
							problem.SetParameterBlockConstant(statesPosUGV[i].parameter);
							problem.SetParameterBlockConstant(statesPosUAV[i].parameter);							
					}
				}
				if(tether_length_constraint){
					ROS_INFO(PRINTF_ORANGE"		- Optimize Tether Length Autodiff - Catenary");
					for (int i = 0; i < statesTetherParams.size(); ++i) {
						CostFunction* cost_function_par_2  = new AutoDiffCostFunction<AutodiffTetherLengthFunctor::TetherLengthFunctor, 1, 4, 4, 4>
									(new AutodiffTetherLengthFunctor::TetherLengthFunctor(w_eta_2, pose_reel_local.transform.translation, length_tether_max, write_data_residual, user_name)); 
							problem.AddResidualBlock(cost_function_par_2, loss_function, statesPosUGV[i].parameter, statesPosUAV[i].parameter, statesTetherParams[i].parameter);
							problem.SetParameterLowerBound(statesTetherParams[i].parameter, 2, 0.01);
							problem.SetParameterLowerBound(statesTetherParams[i].parameter, 3, 0.1);
							problem.SetParameterBlockConstant(statesPosUGV[i].parameter);
							problem.SetParameterBlockConstant(statesPosUAV[i].parameter);
					}		
				}
				if(tether_parameters_constraint){
					ROS_INFO(PRINTF_ORANGE"		- Optimize Tether Params Autodiff - Catenary");
					for (int i = 0; i < statesTetherParams.size(); ++i) {
						CostFunction* cost_function_par_3  = new AutoDiffCostFunction<AutodiffTetherParametersFunctor::TetherParametersFunctor, 2, 4, 4, 4>
									(new AutodiffTetherParametersFunctor::TetherParametersFunctor(w_eta_3, pose_reel_local.transform.translation, write_data_residual, user_name)); 
							problem.AddResidualBlock(cost_function_par_3, loss_function, statesPosUGV[i].parameter, statesPosUAV[i].parameter, statesTetherParams[i].parameter);
							problem.SetParameterLowerBound(statesTetherParams[i].parameter, 2, 0.01);
							problem.SetParameterLowerBound(statesTetherParams[i].parameter, 3, 0.1);
							problem.SetParameterBlockConstant(statesPosUGV[i].parameter);
							problem.SetParameterBlockConstant(statesPosUAV[i].parameter);
					}		
				}
			}
			else{ 
				if(tether_obstacle_constraint){
					ROS_INFO(PRINTF_ORANGE"		- Optimize Tether Obstacles Autodiff - Parabola");
					for (int i = 0; i < statesTetherParams.size(); ++i) {
						CostFunction* cost_function_par_1  = new AutoDiffCostFunction<AutodiffParableFunctor::ParableFunctor, 1, 4, 4, 4> //Residual, ugvPos, uavPos, parableParams
													(new AutodiffParableFunctor::ParableFunctor(w_eta_1, grid_3D, pose_reel_local.transform.translation, distance_tether_obstacle,
													true, user_name)); 
						// CostFunction* cost_function_par_1  = new AutoDiffCostFunction<AutodiffParableFunctor::ParableFunctor, 1, 4, 4, 4> //Residual, ugvPos, uavPos, parableParams
						// 							(new AutodiffParableFunctor::ParableFunctor(w_eta_1, grid_3D, grid_3D_obst, grid_3D_trav, pose_reel_local.transform.translation, distance_tether_obstacle,
						// 							true, user_name)); 
							problem.AddResidualBlock(cost_function_par_1, loss_function, statesPosUGV[i].parameter, statesPosUAV[i].parameter, statesTetherParams[i].parameter);
							problem.SetParameterLowerBound(statesTetherParams[i].parameter, 1, 0.001);
							problem.SetParameterLowerBound(statesTetherParams[i].parameter, 3, pose_reel_local.transform.translation.z);
							problem.SetParameterBlockConstant(statesPosUGV[i].parameter);
							problem.SetParameterBlockConstant(statesPosUAV[i].parameter);
					}
				}
				if(tether_length_constraint){
					ROS_INFO(PRINTF_ORANGE"		- Optimize Tether Length Autodiff - Parabola");
					for (int i = 0; i < statesTetherParams.size(); ++i) {
						CostFunction* cost_function_par_2  = new AutoDiffCostFunction<AutodiffParableLengthFunctor::ParableLengthFunctor, 1, 4, 4, 4>
													(new AutodiffParableLengthFunctor::ParableLengthFunctor(w_eta_2, pose_reel_local.transform.translation, 
													length_tether_max, write_data_residual, user_name)); 
							problem.AddResidualBlock(cost_function_par_2, loss_function, statesPosUGV[i].parameter, statesPosUAV[i].parameter, statesTetherParams[i].parameter);
							problem.SetParameterLowerBound(statesTetherParams[i].parameter, 1, 0.001);
							problem.SetParameterLowerBound(statesTetherParams[i].parameter, 3, pose_reel_local.transform.translation.z);
							problem.SetParameterBlockConstant(statesPosUGV[i].parameter);
							problem.SetParameterBlockConstant(statesPosUAV[i].parameter);
					}		
				}
				if(tether_parameters_constraint){
					ROS_INFO(PRINTF_ORANGE"		- Optimize Tether Params Autodiff - Parabola");
					for (int i = 0; i < statesTetherParams.size(); ++i) {
						CostFunction* cost_function_par_3  = new AutoDiffCostFunction<AutodiffParableParametersFunctor::ParableParametersFunctor, 2, 4, 4, 4>
													(new AutodiffParableParametersFunctor::ParableParametersFunctor(w_eta_3, pose_reel_local.transform.translation, 
													write_data_residual, user_name)); 
							problem.AddResidualBlock(cost_function_par_3, loss_function, statesPosUGV[i].parameter, statesPosUAV[i].parameter, statesTetherParams[i].parameter);
							problem.SetParameterLowerBound(statesTetherParams[i].parameter, 1, 0.001);
							problem.SetParameterLowerBound(statesTetherParams[i].parameter, 3, pose_reel_local.transform.translation.z);
							problem.SetParameterBlockConstant(statesPosUGV[i].parameter);
							problem.SetParameterBlockConstant(statesPosUAV[i].parameter);
					}		
				}
			}

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

	// Inform if is a feabible Trajectory or not, 
	bool free_collision_ = true;
    free_collision_ = CheckStatusTetherCollision(vec_pose_ugv_opt, vec_pose_uav_opt, v_tether_params_opt);

	double time_sleep_ = 2.0;
	bool finished_optimization_ = false;
	if (free_collision_){
		if (initial_cost==final_cost)
			ROS_INFO(PRINTF_ORANGE"\n		Optimizer Local Planner: Final Cost equal than Initial Cost");
		ROS_INFO(PRINTF_GREEN"\n\n\n		Optimizer Local Planner: Goal position successfully achieved through optimization trajectory\n\n\n");

		// Visualize traj in RVIZ
		ros::Duration(time_sleep_).sleep();
		finished_optimization_ = true;
	}
	else{
		if (initial_cost==final_cost)
			ROS_INFO(PRINTF_ORANGE"\n			Optimizer Local Planner: Final Cost equal than Initial Cost");
			ROS_INFO(PRINTF_RED"\n\n\n\n		Optimizer Local Planner: Goal position Not achieved through optimization trajectory\n\n\n");
			ros::Duration(time_sleep_).sleep();

		finished_optimization_ = true;		
	}
	std::cout <<"Optimization Proccess Completed !!!" << std::endl << "Saving Temporal data in txt file ..." << std::endl << "===================================================" << std::endl << std::endl << std::endl;

	ros::Duration(2.0).sleep();

	/********************* To obligate pause method and check Planning result *********************/
        std::string yy_ ;
        std::cout << " *** After Optimization proccess" ;
        std::cout << " : Press key to continue : " ;
        std::cin >> yy_ ;
    /*************************************************************************************************/

	cleanVectors();		//Clear vector after optimization and previus the next iteration
	// Clear optimized Markers
	MP.clearMarkers(catenary_marker, 150, tether_marker_init_pub_);
  	MP.clearMarkersPointLines(points_ugv_marker, lines_ugv_marker,traj_marker_ugv_pub_,size_path);
  	MP.clearMarkersPointLines(points_uav_marker, lines_uav_marker,traj_marker_uav_pub_,size_path);
	MP.clearMarkers(catenary_marker, 150, tether_marker_opt_pub_);
  	MP.clearMarkersPointLines(points_ugv_marker, lines_ugv_marker,traj_opt_marker_ugv_pub_,size_path);
  	MP.clearMarkersPointLines(points_uav_marker, lines_uav_marker,traj_opt_marker_uav_pub_,size_path);
	resetFlags();
}

void TestTetherConstraints::finishigOptimization()
{
	vec_pose_ugv_opt.clear(); 	vec_pose_uav_opt.clear(); 	v_tether_params_opt.clear();
	geometry_msgs::Point position_ugv_, position_uav_;
	tether_parameters param_; 

	double new_equi_dist = 0; 
	double distance_ = 0;
  	for (size_t i = 0; i < size_path; i++){
		position_ugv_.x = statesPosUGV[i].parameter[1];
		position_ugv_.y = statesPosUGV[i].parameter[2];
		position_ugv_.z = statesPosUGV[i].parameter[3];
		vec_pose_ugv_opt.push_back(position_ugv_);
		position_uav_.x = statesPosUAV[i].parameter[1];
		position_uav_.y = statesPosUAV[i].parameter[2];
		position_uav_.z = statesPosUAV[i].parameter[3];
		vec_pose_uav_opt.push_back(position_uav_);
		param_.a = statesTetherParams[i].parameter[1];
		param_.b = statesTetherParams[i].parameter[2];
		param_.c = statesTetherParams[i].parameter[3];
		v_tether_params_opt.push_back(param_);
	}

	graphTetherAndPathMarker(vec_pose_ugv_opt, vec_pose_uav_opt, vec_rot_ugv_opt, v_tether_params_opt, vec_len_tether_opt, 1, 2, 1,
							  traj_opt_marker_ugv_pub_,traj_opt_marker_uav_pub_, tether_marker_opt_pub_, tether_marker_opt);
			  
	for(size_t i = 0; i < size_path; i++){
		printf(PRINTF_REGULAR"Optimized States.Parameter[%lu/%i]: UGV=[%2.3f %2.3f %2.3f] UAV=[%.3f %.3f %.3f] tether=[%.3f %.3f %.3f]\n", 
		i, size_path,
		statesPosUGV[i].parameter[1], statesPosUGV[i].parameter[2], statesPosUGV[i].parameter[3],
		statesPosUAV[i].parameter[1], statesPosUAV[i].parameter[2], statesPosUAV[i].parameter[3],
		v_tether_params_opt[i].a, v_tether_params_opt[i].b, v_tether_params_opt[i].c);
	}
}

void TestTetherConstraints::getReelPose()
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

void TestTetherConstraints::graphTetherAndPathMarker(vector<geometry_msgs::Point> v_ugv_, vector<geometry_msgs::Point> v_uav_, 
													  vector<geometry_msgs::Quaternion> v_rot_ugv_, vector <tether_parameters> v_params_, vector<float> v_length_,
													  int c_ugv_, int c_uav_, int c_tether_, ros::Publisher p_ugv_, ros::Publisher p_uav_, 
													  ros::Publisher p_tether_, visualization_msgs::MarkerArray m_){
	
	MP.getMarkerPoints(points_ugv_marker, v_ugv_, "points_ugv_m",c_ugv_);	// RED= 0 ; GREEN= 1 ; BLUE= 2 ; YELLOW= 3 ; PURPLE= 4; BLACK=5; WHITE=6
	MP.getMarkerLines(lines_ugv_marker, v_ugv_, "lines_ugv_m",c_ugv_);
	MP.getMarkerPoints(points_uav_marker, v_uav_, "points_uav_m",c_uav_);
	MP.getMarkerLines(lines_uav_marker, v_uav_, "lines_uav_m",c_uav_);
	p_ugv_.publish(points_ugv_marker);
	p_ugv_.publish(lines_ugv_marker);
	p_uav_.publish(points_uav_marker);
	p_uav_.publish(lines_uav_marker);
	
	GetTetherParameter GTP_;
	geometry_msgs::Point  p_reel_;
	std::vector<geometry_msgs::Point> v_pts_tether_;

	for(size_t i = 0; i < v_params_.size(); i++){ // The Reel Position is consider above base_link_ugv1
		v_pts_tether_.clear();
		p_reel_.x = v_ugv_[i].x;
		p_reel_.y = v_ugv_[i].y;  
		p_reel_.z = v_ugv_[i].z + pose_reel_local.transform.translation.z;
		GTP_.getParabolaPoints(p_reel_, v_uav_[i], v_params_[i], v_pts_tether_);
		MP.markerPoints(m_, v_pts_tether_, i, v_pts_tether_.size(), p_tether_, c_tether_, print_point_in_graph_marker);	
		/********************* To obligate pause method and check Planning result *********************/
        std::string y_ ;
        std::cout << " *** ["<< i <<"] Optimization Proccess Completed " ;
        std::cout << " : Press key to continue : " ;
        std::cin >> y_ ;
    	/*************************************************************************************************/
	}
}

bool TestTetherConstraints::CheckStatusTetherCollision(vector<geometry_msgs::Point> v1_, vector<geometry_msgs::Point >v2_, vector<tether_parameters> v3_)
{
	geometry_msgs::Point p_reel_; 
	std::vector<geometry_msgs::Point> points_tether_;
	bool ret_;
    double dist_;
	int count_tether_coll, first_coll_, last_coll_, count_total_tether_coll_;
	count_tether_coll = count_total_tether_coll_ = 0;

    std::cout << std::endl << "	CatenaryCheckerManager started: analizing collision status for the marsupial agents" << std::endl; 
	
	double bound_coll_factor = 0.5;
	for (size_t i= 0 ; i <  v1_.size(); i++){
		
		p_reel_.x = v1_[i].x;
		p_reel_.y = v1_[i].y;
		p_reel_.z = v1_[i].z + pose_reel_local.transform.translation.z;

		points_tether_.clear();
		GetTetherParameter GPP_;
		GPP_.getParabolaPoints(p_reel_, v2_[i], v3_[i], points_tether_);

		double dist_safety_;
		count_tether_coll = 0;
		first_coll_ = last_coll_ = -1;
		for (size_t j = 0 ; j < points_tether_.size() ; j++ ) {
            dist_ = CheckCM->getPointDistanceObstaclesMap(true, points_tether_[j],i,"TETHER") ;
			// dist_safety_ = 0.02;
			dist_safety_ =distance_tether_obstacle;
		    // dist_safety_ = distance_tether_obstacle * bound_coll_factor;
		    if( dist_ < dist_safety_){
            	count_tether_coll++;
				if (first_coll_ == -1)
					first_coll_ = j;
				last_coll_ = j;
            	// std::cout << " 		The agent TETHER in the state[" << i << "/"<< v1_.size()<<"] position[" << j <<"/"<< points_tether_.size() <<"] is in COLLISION ["
						// << dist_ <<" mts to obstacle/"<< dist_safety_<<"] pto["<< points_tether_[j].x <<", "<< points_tether_[j].y << ", "<< points_tether_[j].z <<"] reel[" 
						// << p_reel_.x <<"," << p_reel_.y << "," << p_reel_.z <<"] UAV["<< v2_[i].x<<"," <<v2_[i].y <<"," <<v2_[i].z << "]" <<std::endl; 
			}
		}
		if (count_tether_coll>0 )
			std::cout << "TETHER ["<< i <<"] in collision. Total Points:["<< count_tether_coll <<"] , between["<< first_coll_ <<"-"<<last_coll_ <<"]" << std::endl;
		count_total_tether_coll_ = count_total_tether_coll_ + count_tether_coll;
	}

	if (count_total_tether_coll_ > 0){
		ROS_INFO_COND(true, PRINTF_RED "\n \t\tcheckCollisionPathPlanner: Marsupial system in collision for solution [tether=%i]",count_total_tether_coll_);
		ret_ = false;
	}
	else{
		ROS_INFO_COND(true, PRINTF_GREEN "\n \t\tcheckCollisionPathPlanner: Marsupial system collision free for solution [tether=%i]",count_total_tether_coll_);
		ret_ = true;
	}

    std::cout << "	CatenaryCheckerManager finished" << std::endl << std::endl; 

	return ret_;
}
