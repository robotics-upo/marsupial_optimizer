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

	nh->param<double>("td_", td_, 0.5);
		
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
	ROS_INFO("alpha=[%f] beta=[%f] gamma=[%f] delta=[%f] epsilon=[%f] zeta=[%f] eta=[%f] lambda=[%f]",w_alpha,w_beta,w_gamma,w_delta,w_epsilon,w_zeta,w_eta,w_lambda);
	ROS_INFO("Optimizer_Local_Planner: Parameters loaded, ready for Optimization Process. Waiting for Action!!");
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
    octomap_sub_ = nh->subscribe( "/octomap_point_cloud_centers", 1,  &OptimizerLocalPlanner::readOctomapCallback, this);
    ROS_INFO("Optimizer_Local_Planner: Subscribers Initialized");
}

void OptimizerLocalPlanner::initializePublishers()
{
  	traj_marker_ugv_pub_ = nh->advertise<visualization_msgs::MarkerArray>("opt_trajectory_ugv_marker", 2);
  	traj_marker_uav_pub_ = nh->advertise<visualization_msgs::MarkerArray>("opt_trajectory_uav_marker", 2);
	catenary_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("catenary_marker", 100);

  	ROS_INFO("Optimizer_Local_Planner: Publishers Initialized");
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
	  nn_.setInput(*msg);
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
	std::vector<float> length_catenary_initial;
	length_catenary_initial = path_shared_ptr->length_catenary; 
	printf("length_catenary_initial.size=[%lu]\n",length_catenary_initial.size());

	for (size_t i = 0; i < globalTrajectory.points.size(); i++){
		printf("point=[%lu/%lu] GlobalTrayectoryPointTranslation=[%f %f %f / %f %f %f / %f]\n",i+1, globalTrajectory.points.size(),
		globalTrajectory.points.at(i).transforms[0].translation.x, globalTrajectory.points.at(i).transforms[0].translation.y, globalTrajectory.points.at(i).transforms[0].translation.z,
		globalTrajectory.points.at(i).transforms[1].translation.x, globalTrajectory.points.at(i).transforms[1].translation.y, globalTrajectory.points.at(i).transforms[1].translation.z,
		length_catenary_initial[i]);
	}
		
	getPointsFromGlobalPath(globalTrajectory, length_catenary_initial, new_path_ugv, new_path_uav);
    auto size = new_path_uav.size();
    ROS_INFO_COND(debug, PRINTF_GREEN "Number of Vertices to optimize = [%lu] = size",size);

	printf("ugv_pos_reel_catenary=[%f %f %f]\n",ugv_pos_catenary.x,ugv_pos_catenary.y,ugv_pos_catenary.z);
	mp_.clearMarkers(catenary_marker, 150, catenary_marker_pub_);
	// ros::Duration(2.0).sleep();

	count_edges = 0;

  	mp_.clearMarkersPointLines(points_ugv_marker, lines_ugv_marker,traj_marker_ugv_pub_,size);
  	mp_.clearMarkersPointLines(points_uav_marker, lines_uav_marker,traj_marker_uav_pub_,size);
	calculateDistanceVertices(vec_dist_init_ugv, vec_dist_init_uav);
 	getTemporalState(vec_time_init_ugv, vec_time_init_uav);

	writeTemporalDataBeforeOptimization();

	std::cout << std::endl <<  "==================================================="  << std::endl << "Preparing to execute Optimization: Creating Parameter Blocks!!" << std::endl;

  	Problem problem;

	//Set Parameter Blocks from path global information
	statesPos.clear();
	statesRot.clear();
	statesTime.clear();
	statesLength.clear();
	printf("[_]states.parameter=[pos_x  pos_y  pos_z  rot_x  rot_y  rot_z  rot_w  time  length_]\n");
	for (size_t i = 0 ; i < new_path_uav.size(); i ++){
		parameterBlockPos parameter_block_pos;
		parameterBlockRot parameter_block_rot;
		parameterBlockTime parameter_block_time;
		parameterBlockLength parameter_block_length;
		
		parameter_block_pos.parameter[0] = i;
		parameter_block_pos.parameter[1] = vec_pose_init_ugv[i].x();
		parameter_block_pos.parameter[2] = vec_pose_init_ugv[i].y(); 
		parameter_block_pos.parameter[3] = vec_pose_init_ugv[i].z(); 
		parameter_block_pos.parameter[4] = vec_pose_init_uav[i].x();
		parameter_block_pos.parameter[5] = vec_pose_init_uav[i].y(); 
		parameter_block_pos.parameter[6] = vec_pose_init_uav[i].z(); 
		
		parameter_block_rot.parameter[0] = i;
		parameter_block_rot.parameter[1] = vec_rot_ugv[i].x;
		parameter_block_rot.parameter[2] = vec_rot_ugv[i].y; 
		parameter_block_rot.parameter[3] = vec_rot_ugv[i].z; 
		parameter_block_rot.parameter[4] = vec_rot_ugv[i].w;
		parameter_block_rot.parameter[5] = vec_rot_uav[i].x;
		parameter_block_rot.parameter[6] = vec_rot_uav[i].y; 
		parameter_block_rot.parameter[7] = vec_rot_uav[i].z; 
		parameter_block_rot.parameter[8] = vec_rot_uav[i].w; 

		parameter_block_time.parameter[0] = i;
		parameter_block_time.parameter[1] = vec_time_init_ugv[i].time;
		parameter_block_time.parameter[2] = vec_time_init_uav[i].time;

		parameter_block_length.parameter[0] = i;
		parameter_block_length.parameter[1] = vec_len_cat_init[i].length;

		statesPos.push_back(parameter_block_pos);
		statesRot.push_back(parameter_block_rot);
		statesTime.push_back(parameter_block_time);
		statesLength.push_back(parameter_block_length);
		printf("[%lu]states.parameterUGV=[%f %f %f / %f %f %f %f/ %f / %f]\n",i,parameter_block_pos.parameter[1],parameter_block_pos.parameter[2],parameter_block_pos.parameter[3],
															parameter_block_rot.parameter[1],parameter_block_rot.parameter[2],parameter_block_rot.parameter[3],parameter_block_rot.parameter[4],
															parameter_block_time.parameter[1],parameter_block_length.parameter[1]);
		printf("[%lu]states.parameterUAV=[%f %f %f / %f %f %f %f/ %f / %f]\n",i,parameter_block_pos.parameter[4],parameter_block_pos.parameter[5],parameter_block_pos.parameter[6],
															parameter_block_rot.parameter[5],parameter_block_rot.parameter[6],parameter_block_rot.parameter[7],parameter_block_rot.parameter[8],
															parameter_block_time.parameter[2],parameter_block_length.parameter[1]);
	}
    
	/*** Cost Function I : Equidistance constrain ***/
	for (int i = 0; i <  statesPos.size() - 1 ; ++i) {
		CostFunction* cost_function1  = new AutoDiffCostFunction<EquiDistanceFunctor, 2, 7, 7>(new EquiDistanceFunctor(w_alpha*10.0, initial_distance_states_ugv, initial_distance_states_uav)); 
		problem.AddResidualBlock(cost_function1, NULL, statesPos[i].parameter, statesPos[i+1].parameter);
		if (i == 0)
			problem.SetParameterBlockConstant(statesPos[i].parameter);
		if (i == (statesPos.size() - 2))
			problem.SetParameterBlockConstant(statesPos[i+1].parameter);
		problem.SetParameterLowerBound(statesPos[i].parameter, 3, 0.0);
		problem.SetParameterUpperBound(statesPos[i].parameter, 3, 1.0);
		problem.SetParameterLowerBound(statesPos[i].parameter, 6, 0.0);
		problem.SetParameterUpperBound(statesPos[i].parameter, 6, 20.0);
	}

	/*** Cost Function II : Obstacles constrain ***/
	for (int i = 0; i < statesPos.size(); ++i) {
    	CostFunction* cost_function2  = new AutoDiffCostFunction<ObstacleDistanceFunctor::ObstaclesFunctor, 1, 7>
											(new ObstacleDistanceFunctor::ObstaclesFunctor(w_beta*10.0, distance_obstacle, nn_.kdtree, nn_.obs_points)); 
    	problem.AddResidualBlock(cost_function2, NULL, statesPos[i].parameter);
  	}

	/*** Cost Function III-i : UAV Kinematic constrain ***/
	for (int i = 0; i < statesPos.size() - 2; ++i) {
		CostFunction* cost_function3_1  = new AutoDiffCostFunction<KinematicsFunctorUAV, 1, 7, 7, 7>(new KinematicsFunctorUAV(w_gamma, angle_min_traj)); 
		problem.AddResidualBlock(cost_function3_1, NULL, statesPos[i].parameter, statesPos[i+1].parameter, statesPos[i+2].parameter);
	}

	/*** Cost Function III-ii : UGV Kinematic constrain ***/
	for (int i = 0; i < statesPos.size() - 2; ++i) {
		CostFunction* cost_function3_2  = new AutoDiffCostFunction<KinematicsFunctorUGV, 1, 7, 7, 7>(new KinematicsFunctorUGV(w_gamma, angle_min_traj)); 
		problem.AddResidualBlock(cost_function3_2, NULL, statesPos[i].parameter, statesPos[i+1].parameter, statesPos[i+2].parameter);
	}

	/*** Cost Function IV : Time constrain ***/
	for (int i = 0; i < statesTime.size(); ++i) {
		CostFunction* cost_function4  = new AutoDiffCostFunction<TimeFunctor, 2, 3>(new TimeFunctor(w_delta, vec_time_init_ugv[i].time, vec_time_init_uav[i].time)); 
		problem.AddResidualBlock(cost_function4, NULL, statesTime[i].parameter);
		if (i == 0)
			problem.SetParameterBlockConstant(statesTime[i].parameter);
		problem.SetParameterLowerBound(statesTime[i].parameter, 1, 0.0);
	}

	/*** Cost Function V : Velocity constrain ***/
	for (int i = 0; i < statesTime.size() - 1; ++i) {
		CostFunction* cost_function5  = new AutoDiffCostFunction<VelocityFunctor, 2, 7, 7, 3>(new VelocityFunctor(w_epsilon, initial_velocity_ugv, initial_velocity_uav)); 
		problem.AddResidualBlock(cost_function5, NULL, statesPos[i].parameter, statesPos[i+1].parameter, statesTime[i+1].parameter);
	}

	/*** Cost Function VI : Acceleration constrain ***/
	for (int i = 0; i < statesTime.size() - 2; ++i) {
		CostFunction* cost_function6  = new AutoDiffCostFunction<AccelerationFunctor, 2, 7, 7, 7, 3, 3>(new AccelerationFunctor(w_zeta, initial_acceleration_ugv, initial_acceleration_uav)); 
		problem.AddResidualBlock(cost_function6, NULL, statesPos[i].parameter, statesPos[i+1].parameter, statesPos[i+2].parameter, statesTime[i+1].parameter, statesTime[i+2].parameter);
	}

	/*** Cost Function VII : Catenary constrain  ***/
	for (int i = 0; i < statesLength.size(); ++i) {
		CostFunction* cost_function7  = new NumericDiffCostFunction<CatenaryFunctor, CENTRAL, 2, 7, 9, 2>
										(new CatenaryFunctor(w_eta * 10.0, distance_catenary_obstacle, vec_len_cat_init[i].length, nn_.kdtree, nn_.obs_points, pos_reel_ugv, size, nh)); 
		problem.AddResidualBlock(cost_function7, NULL, statesPos[i].parameter, statesRot[i].parameter, statesLength[i].parameter);
		if (i == 0)
			problem.SetParameterBlockConstant(statesLength[i].parameter);
		problem.SetParameterLowerBound(statesLength[i].parameter, 1, 0.0);
		problem.SetParameterUpperBound(statesLength[i].parameter, 1, 40.0);
	}

	/*** Cost Function VIII : Dynamic Catenary constrain  ***/
	// for (int i = 0; i < statesLength.size() - 1; ++i) {
	// 	CostFunction* cost_function8  = new AutoDiffCostFunction<DynamicCatenaryFunctor, 1, 2, 2>(new DynamicCatenaryFunctor(w_lambda, dynamic_catenary)); 
	// 	problem.AddResidualBlock(cost_function8, NULL, statesLength[i].parameter, statesLength[i+1].parameter);
	// }

	std::cout <<  "==================================================="  << std::endl << "Optimization  started !!" << std::endl;

	start_time_opt_ = ros::Time::now();

	ceres::Solve(options, &problem, &summary);
	std::cout << summary.BriefReport() << "\n";
	final_time_opt_ = ros::Time::now();

	std::cout << std::endl <<"Optimization Proccess Completed !!!" << std::endl << "===================================================" << std::endl << "Saving Temporal data in txt file ..." << std::endl;

	vec_pose_ugv_opt.clear();
	vec_pose_uav_opt.clear();
	vec_time_ugv_opt.clear();
	vec_time_uav_opt.clear();
	vec_len_cat_opt.clear();
	vec_dist_ugv_opt.clear();
	vec_dist_uav_opt.clear();
	vec_vel_ugv_opt.clear();
	vec_vel_uav_opt.clear();
	vec_acc_uav_opt.clear();
	new_path_ugv.clear(); //clear the old path to set the optimizer solution as a new path
	new_path_uav.clear(); //clear the old path to set the optimizer solution as a new path
	

  	for (size_t i = 0; i < size; i++){
		Eigen::Vector3d position_ugv_ = Eigen::Vector3d(statesPos[i].parameter[1],statesPos[i].parameter[2],statesPos[i].parameter[3]);
		vec_pose_ugv_opt.push_back(position_ugv_);
		Eigen::Vector3d position_uav_ = Eigen::Vector3d(statesPos[i].parameter[4],statesPos[i].parameter[5],statesPos[i].parameter[6]);
		vec_pose_uav_opt.push_back(position_uav_);
		structLengthCatenary vLC;
		vLC.length = statesLength[i].parameter[1];
		vLC.id = i;
		vec_len_cat_opt.push_back(vLC);
		vec_time_ugv_opt.push_back(statesTime[i].parameter[1]);
		vec_time_uav_opt.push_back(statesTime[i].parameter[2]);
	}

	// mp_.clearMarkers(catenary_marker, 100, catenary_marker_pub_);
	for(size_t i = 0; i < statesPos.size(); i++){
		std::vector<geometry_msgs::Point> points_catenary_final;
		points_catenary_final.clear();

		geometry_msgs::Point p_reel_=getReelPoint(statesPos[i].parameter[1],statesPos[i].parameter[2],statesPos[i].parameter[3],statesRot[i].parameter[1],statesRot[i].parameter[2],statesRot[i].parameter[3], statesRot[i].parameter[4],pos_reel_ugv);
		CatenarySolver cSX_;
		cSX_.setMaxNumIterations(100);
	  	cSX_.solve(p_reel_.x, p_reel_.y, p_reel_.z, statesPos[i].parameter[4], statesPos[i].parameter[5], statesPos[i].parameter[6], statesLength[i].parameter[1], points_catenary_final);
		mp_.markerPoints(catenary_marker, points_catenary_final, i, size, catenary_marker_pub_);
		double _d_ = sqrt(pow(p_reel_.x -statesPos[i].parameter[4],2) + pow(p_reel_.y - statesPos[i].parameter[5],2) + pow(p_reel_.z-statesPos[i].parameter[6],2));
		printf("points_catenary_final.size()=[%lu] ugv_pos[%lu] = [%f %f %f] , uav_pos[%lu]=[%f %f %f] , l=[%f] d=[%f]\n",points_catenary_final.size(),
																												i,statesPos[i].parameter[1], statesPos[i].parameter[2], statesPos[i].parameter[3],
																												i,statesPos[i].parameter[4], statesPos[i].parameter[5], statesPos[i].parameter[6],
																												statesLength[i].parameter[1],_d_);
	}

	mp_.getMarkerPoints(points_ugv_marker, vec_pose_ugv_opt, "points_ugv_m",1);
	mp_.getMarkerLines(lines_ugv_marker, vec_pose_ugv_opt, "lines_ugv_m",1);
	mp_.getMarkerPoints(points_uav_marker, vec_pose_uav_opt, "points_uav_m",1);
	mp_.getMarkerLines(lines_uav_marker, vec_pose_uav_opt, "lines_uav_m",1);
	traj_marker_ugv_pub_.publish(points_ugv_marker);
	traj_marker_ugv_pub_.publish(lines_ugv_marker);
	traj_marker_uav_pub_.publish(points_uav_marker);
	traj_marker_uav_pub_.publish(lines_uav_marker);

	writeTemporalDataAfterOptimization(size);

	ros::Duration(td_).sleep();

	output_file = path+name_output_file+"_stage_"+std::to_string(scenario_number)+"_InitPos_"+std::to_string(num_pos_initial)+"_goal_"+std::to_string(num_goal)+".txt";
	ofs.open(output_file.c_str(), std::ofstream::app);
	if (write_data_for_analysis)
		getDataForOptimizerAnalysis();

	cleanVectors();		//Clear vector after optimization and previus the next iteration

	std::cout << "...Temporal data saved in txt file." << std::endl << std::endl;
	ROS_INFO("Optimizer Local Planner: Goal position successfully achieved");
	ros::Duration(5.0).sleep();
	action_result.arrived = true;
	execute_path_srv_ptr->setSucceeded(action_result);

	std::cout <<"Optimization Local Planner Proccess ended !!!!!!!!!!!!!!!!!!!!!!!" << std::endl << "==================================================="<< std::endl << std::endl;
	resetFlags();

	for (size_t j = 0; j < vec_len_cat_opt.size(); j++){
		printf("Optimized length_catenary=[%f] for state=[%lu]\n",vec_len_cat_opt[j].length,j);
	}

  std::cout << "==================================================="<< std::endl << std::endl;
}

void OptimizerLocalPlanner::getDataForOptimizerAnalysis()
{
	// bisectionCatenary bsCat;
	std::vector<geometry_msgs::Point> v_points_catenary_opt_,v_points_catenary_init_;

	// Writing general data for initial analysis
	double init_compute_time_, init_traj_distance_, init_traj_time_, init_traj_vel_, init_traj_vel_max_, init_traj_vel_mean_, init_traj_acc_,init_traj_acc_max_,init_traj_acc_mean_;
	init_compute_time_ = init_traj_distance_ = init_traj_time_ = init_traj_vel_ = init_traj_vel_max_ = init_traj_vel_mean_ = init_traj_acc_ = init_traj_acc_max_ = init_traj_acc_mean_=0.0;

	//Length and Time Trajectory Initial
	for(size_t i = 0 ; i < vec_time_init_uav.size()-1 ; i++){
		init_traj_time_ = vec_time_init_uav[i+1].time + init_traj_time_;
		init_traj_distance_ = vec_dist_init_uav[i].dist + init_traj_distance_;
		init_traj_vel_ = (vec_dist_init_uav[i].dist / vec_time_init_uav[i+1].time ) + init_traj_vel_; 
		if (init_traj_vel_max_ < (vec_dist_init_uav[i].dist / vec_time_init_uav[i+1].time))
			init_traj_vel_max_ = (vec_dist_init_uav[i].dist / vec_time_init_uav[i+1].time);
	}
	init_traj_vel_mean_ = init_traj_vel_ / ((double)vec_time_init_uav.size()-1.0);

	//Acceleration Trajectory Initial
	for(size_t i = 0 ; i < vec_time_init_uav.size()-2 ; i++){
		init_traj_acc_ = ( ((vec_dist_init_uav[i].dist / vec_time_init_uav[i+1].time ) - (vec_dist_init_uav[i+1].dist/vec_time_init_uav[i+2].time)) / (vec_time_init_uav[i+1].time +vec_time_init_uav[i+2].time) ) + init_traj_acc_; ;
		if (init_traj_acc_max_ < ( ((vec_dist_init_uav[i].dist / vec_time_init_uav[i+1].time )- (vec_dist_init_uav[i+1].dist/vec_time_init_uav[i+2].time)) / (vec_time_init_uav[i+1].time +vec_time_init_uav[i+2].time) ) )
			init_traj_acc_max_ = ( ((vec_dist_init_uav[i].dist / vec_time_init_uav[i+1].time )- (vec_dist_init_uav[i+1].dist/vec_time_init_uav[i+2].time)) / (vec_time_init_uav[i+1].time +vec_time_init_uav[i+2].time) );
	}
	init_traj_acc_mean_ = init_traj_acc_ / (double)(vec_time_init_uav.size()-2.0);

	//Distance Obstacles Initial
	double distance_obs_init_ , distance_obs_init_min_, distance_obs_init_mean_;
	distance_obs_init_ = distance_obs_init_mean_ = 0.0;
	distance_obs_init_min_ = 1000.0;
	for(size_t i = 0 ; i < vec_pose_init_uav.size(); i ++){
		Eigen::Vector3d p_init_ = vec_pose_init_uav[i];
		Eigen::Vector3d nearest_obs_p_ = nn_.nearestObstacleVertex(nn_.kdtree, p_init_ , nn_.obs_points);
		distance_obs_init_ = (p_init_- nearest_obs_p_).norm() + distance_obs_init_;
		if(distance_obs_init_min_ > (p_init_- nearest_obs_p_).norm() )
			distance_obs_init_min_ = (p_init_- nearest_obs_p_).norm();
	}
	distance_obs_init_mean_ = distance_obs_init_ / (double)vec_pose_init_uav.size();

	//Distance Catenary Obstacles Initial
	double distance_obs_cat_init_ , distance_obs_cat_init_min_, distance_obs_cat_init_mean_;
	distance_obs_cat_init_ = distance_obs_cat_init_mean_ = 0.0;
	distance_obs_cat_init_min_ = 1000.0;
	int count_cat_p_init_ = 0;
	for(size_t i = 0 ; i < vec_pose_init_uav.size(); i ++){
		CatenarySolver cS_;
		v_points_catenary_init_.clear();
		cS_.setMaxNumIterations(100);
		cS_.solve(ugv_pos_catenary.x, ugv_pos_catenary.y, ugv_pos_catenary.z, vec_pose_init_uav[i].x(), vec_pose_init_uav[i].y(), vec_pose_init_uav[i].z(), vec_len_cat_init[i].length, v_points_catenary_init_);
		int n_p_cat_dis_ = ceil(1.5*ceil(vec_len_cat_init[i].length)); // parameter to ignore collsion points in the begining and in the end of catenary
		if (n_p_cat_dis_ < 5)
			n_p_cat_dis_ = 5;
		for (size_t j= 0 ; j < v_points_catenary_init_.size() ; j++){
			if(j > n_p_cat_dis_ && j < v_points_catenary_init_.size()-floor(n_p_cat_dis_/2.0)){
				count_cat_p_init_++;
				Eigen::Vector3d p_cat_init_; 
				p_cat_init_.x()= v_points_catenary_init_[j].x;
				p_cat_init_.y()= v_points_catenary_init_[j].y;
				p_cat_init_.z()= v_points_catenary_init_[j].z;
				Eigen::Vector3d nearest_obs_p = nn_.nearestObstacleVertex(nn_.kdtree, p_cat_init_ , nn_.obs_points);
				distance_obs_cat_init_ = (p_cat_init_- nearest_obs_p).norm() + distance_obs_cat_init_;
				if(distance_obs_cat_init_min_ > (p_cat_init_- nearest_obs_p).norm() )
					distance_obs_cat_init_min_ = (p_cat_init_- nearest_obs_p).norm();
			}
		}
	}
	distance_obs_cat_init_mean_ = distance_obs_cat_init_/ (double)count_cat_p_init_;

	
	// Writing general data for optimized analysis
	double opt_compute_time_, opt_traj_distance_, opt_traj_time_, opt_traj_vel_, opt_traj_vel_max_, opt_traj_vel_mean_, opt_traj_acc_, opt_traj_acc_max_, opt_traj_acc_mean_;
	opt_compute_time_ = opt_traj_distance_ = opt_traj_time_ = opt_traj_vel_ = opt_traj_vel_max_ = opt_traj_vel_mean_ = opt_traj_acc_ = opt_traj_acc_max_ = opt_traj_acc_mean_ = 0.0;

	//Compute Time Optimization
	opt_compute_time_ = (final_time_opt_ - start_time_opt_).toSec();

	//Length, Time, Velocity Trajectory Optimized
	for (size_t i = 0; i < vec_time_uav_opt.size() -1 ; i++){
		opt_traj_time_ = vec_time_uav_opt[i+1] + opt_traj_time_;
		opt_traj_distance_ = vec_dist_uav_opt[i] + opt_traj_distance_;
		opt_traj_vel_ = vec_vel_uav_opt[i] + opt_traj_vel_;
		if (opt_traj_vel_max_ < vec_vel_uav_opt[i])
			opt_traj_vel_max_ = vec_vel_uav_opt[i];
	}
	opt_traj_vel_mean_ = opt_traj_vel_ / ((double)vec_time_uav_opt.size()-1.0);

	//Acceleration Trajectory Optimized
	for (size_t i = 0; i < vec_acc_uav_opt.size() ; i++){
		opt_traj_acc_ = vec_acc_uav_opt[i] + opt_traj_acc_;
		if (fabs(opt_traj_acc_max_) < fabs(vec_acc_uav_opt[i]))
			opt_traj_acc_max_ = vec_acc_uav_opt[i];
	}
	opt_traj_acc_mean_ = opt_traj_acc_ / (double)vec_acc_uav_opt.size();

	//Distance Point Obstacles
	double distance_obs_opt_ , distance_obs_opt_min_, distance_obs_opt_mean_;
	distance_obs_opt_ = distance_obs_opt_mean_ = 0.0;
	distance_obs_opt_min_ = 1000.0;
	for(size_t i = 0 ; i < vec_pose_uav_opt.size(); i ++){
		Eigen::Vector3d p_opt_ = vec_pose_uav_opt[i];
		Eigen::Vector3d nearest_obs_p_ = nn_.nearestObstacleVertex(nn_.kdtree, p_opt_ , nn_.obs_points);
		distance_obs_opt_ = (p_opt_- nearest_obs_p_).norm() + distance_obs_opt_;
		if(distance_obs_opt_min_ > (p_opt_- nearest_obs_p_).norm() )
			distance_obs_opt_min_ = (p_opt_- nearest_obs_p_).norm();
	}
	distance_obs_opt_mean_ = distance_obs_opt_ / (double)vec_pose_uav_opt.size();

	//Distance Catenary Obstacles Optimized
	double distance_obs_cat_opt_ , distance_obs_cat_opt_min_, distance_obs_cat_opt_mean_;
	distance_obs_cat_opt_ = distance_obs_cat_opt_mean_ = 0.0;
	distance_obs_cat_opt_min_ = 1000.0;
	int count_cat_p_opt_ = 0;

	for(size_t i = 0 ; i < vec_pose_uav_opt.size(); i ++){
		CatenarySolver cS_;
		v_points_catenary_opt_.clear();
		cS_.setMaxNumIterations(100);
		cS_.solve(ugv_pos_catenary.x, ugv_pos_catenary.y, ugv_pos_catenary.z, vec_pose_uav_opt[i].x(), vec_pose_uav_opt[i].y(), vec_pose_uav_opt[i].z(), vec_len_cat_opt[i].length, v_points_catenary_opt_);
		int n_p_cat_dis_ = ceil(1.5*ceil(vec_len_cat_opt[i].length)); // parameter to ignore collsion points in the begining and in the end of catenary
		if (n_p_cat_dis_ < 5)
			n_p_cat_dis_ = 5;
		for (size_t j= 0 ; j < v_points_catenary_opt_.size() ; j++){
			if(j > n_p_cat_dis_ && j < v_points_catenary_opt_.size()-floor(n_p_cat_dis_/2.0)){
				count_cat_p_opt_++;
				Eigen::Vector3d p_cat_opt_; 
				p_cat_opt_.x()= v_points_catenary_opt_[j].x;
				p_cat_opt_.y()= v_points_catenary_opt_[j].y;
				p_cat_opt_.z()= v_points_catenary_opt_[j].z;
				Eigen::Vector3d nearest_obs_p = nn_.nearestObstacleVertex(nn_.kdtree, p_cat_opt_ , nn_.obs_points);
				distance_obs_cat_opt_ = (p_cat_opt_- nearest_obs_p).norm() + distance_obs_cat_opt_;
				// if ((p_cat_opt_- nearest_obs_p).norm() > 100.0)
				// 	printf ( "state[%lu]=[%f %f %f] point_cat[%lu]=[%f %f %f]\n",i,vec_pose_uav_opt[i].x(), vec_pose_uav_opt[i].y(), vec_pose_uav_opt[i].z(),j,p_cat_opt_.x(),p_cat_opt_.y(),p_cat_opt_.z());
				if(distance_obs_cat_opt_min_ > (p_cat_opt_- nearest_obs_p).norm() )
					distance_obs_cat_opt_min_ = (p_cat_opt_- nearest_obs_p).norm();
			}
		// printf("Obstacles Optimized: state=[%lu/%lu] vec_pose_uav_opt.size=[%lu] v_points_catenary_opt_.size=[%lu] count_cat_p_opt = [%i]\n",i, vec_pose_uav_opt.size(),vec_pose_uav_opt.size(),v_points_catenary_opt_.size(),count_cat_p_opt_);
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

void OptimizerLocalPlanner::writeTemporalDataAfterOptimization(auto _s)
{
	//! Save Temporal Data Optimized in File.txt 
	file_out_time.open (path+"optimized_time.txt");
	file_out_velocity.open (path+"optimized_velocity.txt");
	file_out_acceleration.open (path+"optimized_acceleration.txt");
	double sum_dis_pos_ugv_ = 0.0;
	double sum_dis_pos_uav_ = 0.0;
	double new_vel_ugv_ = 0.0;
	double new_vel_uav_ = 0.0;
	double sum_difftime_ugv_ = 0.0;
	double sum_difftime_uav_ = 0.0;
	for (size_t i=0; i < _s -1; ++i){
		if ( i == 0)
			file_out_time << setprecision(6) << sum_dis_pos_ugv_ << ";" << vec_time_ugv_opt[i] << ";" << sum_dis_pos_uav_ << ";" << vec_time_uav_opt[i] << endl;
		double difftime_ugv_ = vec_time_ugv_opt[i+1];
		double difftime_uav_ = vec_time_uav_opt[i+1];
		double dist_ugv_ = (vec_pose_ugv_opt[i] - vec_pose_ugv_opt[i+1]).norm();
		double dist_uav_ = (vec_pose_uav_opt[i] - vec_pose_uav_opt[i+1]).norm();
		vec_dist_ugv_opt.push_back(dist_ugv_);
		vec_dist_uav_opt.push_back(dist_uav_);
		sum_dis_pos_ugv_ = dist_ugv_ + sum_dis_pos_ugv_;
		sum_dis_pos_uav_ = dist_uav_ + sum_dis_pos_uav_;
		new_vel_ugv_ = dist_ugv_ / difftime_ugv_;
		new_vel_uav_ = dist_uav_ / difftime_uav_;
		vec_vel_ugv_opt.push_back(new_vel_ugv_);
		vec_vel_uav_opt.push_back(new_vel_uav_);
		sum_difftime_ugv_ = sum_difftime_ugv_ + difftime_ugv_;
		sum_difftime_uav_ = sum_difftime_uav_ + difftime_uav_;
		file_out_time << setprecision(6) << sum_dis_pos_ugv_ << ";" << sum_difftime_ugv_ << ";" << sum_dis_pos_uav_ << ";" << sum_difftime_uav_ << endl;
		file_out_velocity << setprecision(6) << sum_dis_pos_ugv_ << ";" << new_vel_ugv_ << ";" << sum_dis_pos_uav_ << ";" << new_vel_uav_ << endl;
	}
	double sum_dis_ugv_ = 0.0;
	double sum_dis_uav_ = 0.0;
	for (size_t i = 0; i < _s-2; i++)
	{
		double difftime1_ugv_ = vec_time_ugv_opt[i+1];
		double difftime2_ugv_ = vec_time_ugv_opt[i+2];
		double difftime1_uav_ = vec_time_uav_opt[i+1];
		double difftime2_uav_ = vec_time_uav_opt[i+2];

		double distance1_ugv_ = (vec_pose_ugv_opt[i+1] - vec_pose_ugv_opt[i]).norm();	
		double distance2_ugv_ = (vec_pose_ugv_opt[i+2] - vec_pose_ugv_opt[i+1]).norm();
		double distance1_uav_ = (vec_pose_uav_opt[i+1] - vec_pose_uav_opt[i]).norm();	
		double distance2_uav_ = (vec_pose_uav_opt[i+2] - vec_pose_uav_opt[i+1]).norm();
		if (i==0){
			sum_dis_ugv_ = distance1_ugv_;
			sum_dis_uav_ = distance1_uav_;
		}
		sum_dis_ugv_ = sum_dis_ugv_ + distance2_ugv_;
		sum_dis_uav_ = sum_dis_uav_ + distance2_uav_;
		
		double sumTime_ugv_ = difftime1_ugv_ + difftime2_ugv_;
		double velocity1_ugv_ = distance1_ugv_ / difftime1_ugv_;
		double velocity2_ugv_ = distance2_ugv_ / difftime2_ugv_;
		double acceleration_ugv_ = (velocity2_ugv_- velocity1_ugv_)/sumTime_ugv_;
		vec_acc_ugv_opt.push_back(acceleration_ugv_);

		double sumTime_uav_ = difftime1_uav_ + difftime2_uav_;
		double velocity1_uav_ = distance1_uav_ / difftime1_uav_;
		double velocity2_uav_ = distance2_uav_ / difftime2_uav_;
		double acceleration_uav_ = (velocity2_uav_- velocity1_uav_)/sumTime_uav_;
		vec_acc_uav_opt.push_back(acceleration_uav_);

		file_out_acceleration << setprecision(6) << sum_dis_ugv_ << ";" << acceleration_ugv_ << ";" << sum_dis_uav_ << ";" << acceleration_uav_ << endl;
	}
	file_out_time.close();
	file_out_velocity.close();
	file_out_acceleration.close();
}

void OptimizerLocalPlanner::writeTemporalDataBeforeOptimization(void){
	//! Save temporal state before optimization
	file_in_time.open (path+"initial_time.txt");
	file_in_velocity.open (path+"initial_velocity.txt");
	file_in_acceleration.open (path+"initial_acceleration.txt");
	
	double _sum_dist_ugv = 0.0;
	double _sum_time_ugv = 0.0;
	double _sum_dist_uav = 0.0;
	double _sum_time_uav = 0.0;

	for (size_t i = 0; i < vec_time_init_uav.size() - 1; i++){
		_sum_dist_ugv = _sum_dist_ugv + vec_dist_init_ugv[i].dist;
		_sum_time_ugv = _sum_time_ugv + vec_time_init_ugv[i+1].time;
		_sum_dist_uav = _sum_dist_uav + vec_dist_init_uav[i].dist;
		_sum_time_uav = _sum_time_uav + vec_time_init_uav[i+1].time;

		file_in_time << setprecision(6) << _sum_dist_ugv << ";" << _sum_time_ugv << ";" << _sum_dist_uav << ";" << _sum_time_uav << endl;
		file_in_velocity  << setprecision(6) << _sum_dist_ugv << ";" << initial_velocity_ugv << ";" << _sum_dist_uav << ";" << initial_velocity_uav << endl;
		if (i > 0)
			file_in_acceleration << setprecision(6) << _sum_dist_ugv << ";" << initial_acceleration_ugv << ";" << _sum_dist_uav << ";" << initial_acceleration_uav << endl;
	}
	file_in_time.close();	
	file_in_velocity.close();	
	file_in_acceleration.close();
}

void OptimizerLocalPlanner::getPointsFromGlobalPath(trajectory_msgs::MultiDOFJointTrajectory _path, vector<float> l_cat_, vector<Eigen::Vector3d> &v_ugv_, vector<Eigen::Vector3d> &v_uav_)
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
			// for (int j=0; j< n_uav_ ; j++){
			// 	// Create new position and rotation vector for UGV
			// 	p_ugv_ = Eigen::Vector3d(0.0,0.0,0.0);
			// 	p_ugv_.x() = _path.points.at(i).transforms[0].translation.x + xp_ugv_*(1.0+(double)j);
			// 	p_ugv_.y() = _path.points.at(i).transforms[0].translation.y + yp_ugv_*(1.0+(double)j);
			// 	p_ugv_.z() = _path.points.at(i).transforms[0].translation.z + zp_ugv_*(1.0+(double)j);
			// 	v_ugv_.push_back(p_ugv_);
			// 	qt_.x = _path.points.at(i).transforms[0].rotation.x;
			// 	qt_.y = _path.points.at(i).transforms[0].rotation.y;
			// 	qt_.z = _path.points.at(i).transforms[0].rotation.z;
			// 	qt_.w = _path.points.at(i).transforms[0].rotation.w;
			// 	vec_rot_ugv.push_back(qt_);
			// 	printf("point =[%i] getPointsFromGlobalPathUGV = [%f %f %f]\n",count_,p_ugv_.x(),p_ugv_.y(),p_ugv_.z());
			// 	// Create new position and rotation vector for UAV
			// 	p_uav_ = Eigen::Vector3d(0.0,0.0,0.0);
			// 	p_uav_.x() = _path.points.at(i).transforms[1].translation.x + xp_uav_*(1.0+(double)j);
			// 	p_uav_.y() = _path.points.at(i).transforms[1].translation.y + yp_uav_*(1.0+(double)j);
			// 	p_uav_.z() = _path.points.at(i).transforms[1].translation.z + zp_uav_*(1.0+(double)j);
			// 	v_uav_.push_back(p_uav_);
			// 	qt_.x = _path.points.at(i).transforms[1].rotation.x;
			// 	qt_.y = _path.points.at(i).transforms[1].rotation.y;
			// 	qt_.z = _path.points.at(i).transforms[1].rotation.z;
			// 	qt_.w = _path.points.at(i).transforms[1].rotation.w;
			// 	vec_rot_uav.push_back(qt_);
			// 	printf("point =[%i] getPointsFromGlobalPathUAV = [%f %f %f]\n",count_,p_uav_.x(),p_uav_.y(),p_uav_.z());
			// 	count_++;
			// }
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


	for(size_t i= 0; i < l_cat_.size() ; i++){
		structLengthCatenary l_;
		l_.id = i;
		l_.length = l_cat_[i];
		vec_len_cat_init.push_back(l_); 
	}

}

void OptimizerLocalPlanner::calculateDistanceVertices(vector<structDistance> &_v_D_ugv,vector<structDistance> &_v_D_uav)
{
	structDistance _sD;
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
		_sD.id_from = i;
		_sD.id_to = i+ 1;
		_sD.dist = pL;
		_v_D_ugv.push_back(_sD);
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
		_sD.id_from = i;
		_sD.id_to = i+ 1;
		_sD.dist = pL;
		_v_D_uav.push_back(_sD);
    }
	initial_distance_states_uav = sum_distance_ / (new_path_uav.size()-1);
}

void OptimizerLocalPlanner::getTemporalState(vector<structTime> &_time_ugv, vector<structTime> &_time_uav) 
{
	structTime _sT;
	_time_ugv.clear();
	_time_uav.clear();
	double _dT = 0;
	for (size_t i= 0 ; i < new_path_ugv.size(); i++){
		_sT.id = i;
		if (i == 0){
			_sT.time = 0.0;
			_time_ugv.push_back(_sT);
		}
		else{
			_dT = (vec_dist_init_ugv[i-1].dist)/initial_velocity_ugv;
			_sT.time = _dT;
			_time_ugv.push_back(_sT);
		}
	}
	_dT = 0;
	for (size_t i= 0 ; i < new_path_uav.size(); i++){
		_sT.id = i;
		if (i == 0){
			_sT.time = 0.0;
			_time_uav.push_back(_sT);
		}
		else{
			_dT = (vec_dist_init_uav[i-1].dist)/initial_velocity_uav;
			_sT.time = _dT;
			_time_uav.push_back(_sT);
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

// void OptimizerLocalPlanner::setInitialLengthCatenaryAndPosUGV(std::vector <double> &_vector, auto _s)
// {
// 	tfListener();
// 	_vector.clear();
// 	for (size_t i = 0; i < _s; i++){
// 		double _dist_X_c_v = (ugv_pos_catenary.x - new_path_uav[i].x());
// 		double _dist_Y_c_v = (ugv_pos_catenary.y - new_path_uav[i].y());
// 		double _dist_Z_c_v = (ugv_pos_catenary.z - new_path_uav[i].z());
// 		double _dist_c_v = sqrt(_dist_X_c_v * _dist_X_c_v + _dist_Y_c_v * _dist_Y_c_v + _dist_Z_c_v * _dist_Z_c_v)*initial_multiplicative_factor_length_catenary;  
// 		_vector.push_back(_dist_c_v);
// 	}
// }

// void OptimizerLocalPlanner::tfListener(){
// 	tf::StampedTransform transform;
//     listener.lookupTransform("/map", "/reel_base_link", ros::Time(0), transform);
// 	ugv_pos_catenary.x = transform.getOrigin().x();
// 	ugv_pos_catenary.y = transform.getOrigin().y();
// 	ugv_pos_catenary.z = transform.getOrigin().z();
// }

// void OptimizerLocalPlanner::straightTrajectoryVertices(double x1, double y1, double z1, double x2, double y2, double z2, int _n_v_u, std::vector<Eigen::Vector3d> &_v)
// {
// 	double _x, _y, _z;
// 	double _d= sqrt(pow(x2-x1,2)+pow(y2-y1,2)+pow(z2-z1,2));
// 	double distance_xy = sqrt(pow(x2-x1,2)+pow(y2-y1,2));
// 	double distance_xz = sqrt(pow(x2-x1,2)+pow(z2-z1,2));
// 	if (distance_xy < 0.00001)
// 		distance_xy = 0.00001;
// 	if (distance_xz < 0.00001)
// 		distance_xz = 0.00001;
// 	double interval_xy = distance_xy/_n_v_u;		// Size of the interval between points in axes xy
// 	double interval_xz = distance_xz/_n_v_u;		// Size of the interval between points in axes xz
// 	for(int i = 1 ; i< _n_v_u+1 ; i++)
// 	{
// 		_x = x1 + i*interval_xy * ((x2-x1)/distance_xy);
// 		_y = y1 + i*interval_xy * ((y2-y1)/distance_xy);
// 		_z = z1 + i*interval_xz * ((z2-z1)/distance_xz);
// 		_v.push_back(Eigen::Vector3d(_x,_y,_z));
// 	}
// }
