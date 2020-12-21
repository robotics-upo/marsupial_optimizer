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
	nh->param<double>("w_gamma", w_gamma,0.1);
	nh->param<double>("w_delta", w_delta,0.1);
	nh->param<double>("w_epsilon", w_epsilon,0.1);
	nh->param<double>("w_zeta", w_zeta,0.1);
	nh->param<double>("w_eta", w_eta,0.1);

  	nh->param<double>("initial_multiplicative_factor_length_catenary", initial_multiplicative_factor_length_catenary,1.0); //Can't be lower than 1.0 or "catenary < distance between UGV and state" 	

	nh->param<int>("n_iter_opt", n_iter_opt,200);
	nh->param<double>("distance_obstacle", distance_obstacle,2.0);
	nh->param<double>("initial_velocity", initial_velocity,1.0);
	nh->param<double>("initial_acceleration", initial_acceleration,0.0);
	nh->param<double>("angle_min_traj", angle_min_traj, M_PI / 15.0);
	nh->param<double>("distance_catenary_obstacle", distance_catenary_obstacle, 0.1);
	nh->param<double>("dynamic_catenary", dynamic_catenary, 0.5);
	nh->param<double>("min_distance_add_new_point", min_distance_add_new_point, 0.5);
	nh->param<double>("bound_bisection_a", bound_bisection_a,100.0);
	nh->param<double>("bound_bisection_b", bound_bisection_b,100.0);

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
		

	initializeSubscribers();
	initializePublishers();
	setupOptimizer();
	cleanVectors();
	ROS_INFO("alpha=[%f] beta=[%f] gamma=[%f] delta=[%f] epsilon=[%f] zeta=[%f] eta=[%f]",w_alpha,w_beta,w_gamma,w_delta,w_epsilon,w_zeta,w_eta);
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

  ROS_INFO("Optimizer_Local_Planner: Publishers Initialized");
}

void OptimizerLocalPlanner::resetFlags()
{
  mapReceived = false;
	continue_optimizing = false;
}

void OptimizerLocalPlanner::cleanVectors()
{
	new_path.clear();
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
}

void OptimizerLocalPlanner::setupOptimizer()
{
  options.max_num_iterations = n_iter_opt;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;
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
}

void OptimizerLocalPlanner::executeOptimizerPathPreemptCB()
{
    ROS_INFO_COND(debug, "Goal Preempted");
    execute_path_srv_ptr->setPreempted(); // set the action state to preempted

    resetFlags();
    mp_.clearMarkersPointLines(points_marker, lines_marker,traj_marker_pub_,0);
}


void OptimizerLocalPlanner::executeOptimizerPathGoalCB()
{
  	ROS_INFO_COND(debug, PRINTF_GREEN "Optimizer Local Planner Goal received in action server mode");
  	auto path_shared_ptr = execute_path_srv_ptr->acceptNewGoal();
  	globalTrajectory = path_shared_ptr->path;
	vector<float> length_catenary_initial;
	length_catenary_initial = path_shared_ptr->length_catenary;
	printf("length_catenary_initial.size=[%lu]\n",length_catenary_initial.size());

	for (size_t i = 0; i < globalTrajectory.points.size(); i++){
		printf("point=[%lu/%lu] GlobalTrayectoryPointTranslation=[%f %f %f]\n",i+1, globalTrajectory.points.size(),
		globalTrajectory.points.at(i).transforms[0].translation.x, globalTrajectory.points.at(i).transforms[0].translation.y, globalTrajectory.points.at(i).transforms[0].translation.z);
	}
		
	getPointsFromGlobalPath(globalTrajectory,new_path);
    auto size = new_path.size();
    ROS_INFO_COND(debug, PRINTF_GREEN "Number of Vertices to optimize = [%lu] = size",size);

	// setInitialLengthCatenaryAndPosUGV(vec_len_cat_init[i].length, size);
	// ros::Duration(4.0).sleep();
	preComputeLengthCatenary(v_pre_initial_length_catenary, size);
	printf("ugv_pos_reel_catenary=[%f %f %f]\n",ugv_pos_catenary.x,ugv_pos_catenary.y,ugv_pos_catenary.z);
	checkObstaclesBetweenCatenaries(v_pre_initial_length_catenary,size);
	ros::Duration(1.0).sleep();
	mp_.clearMarkers(catenary_marker, 150, catenary_marker_pub_);
	ros::Duration(2.0).sleep();

	count_edges = 0;

	// resetFlags();
  	mp_.clearMarkersPointLines(points_marker, lines_marker,traj_marker_pub_,size);
	calculateDistanceVertices(new_path,vec_dist_init);
 	getTemporalState(vec_time_init,vec_dist_init,initial_velocity);

	writeTemporalDataBeforeOptimization();

	if (!continue_optimizing){
		std::cout << std::endl <<  "==================================================="  << std::endl 
		<< "Preparing to execute Optimization: Creating Vertices and Edges!!" << std::endl;
	}

  	Problem problem;

	statesPos.clear();
	statesTime.clear();
	statesLength.clear();
	printf("[_]states.parameter=[_positi_x _positi_y _positi_z __time__ _length_]\n");
	for (size_t i = 0 ; i < new_path.size(); i ++){
		parameterBlockPos parameter_block_pos;
		parameterBlockTime parameter_block_time;
		parameterBlockLength parameter_block_length;
		parameter_block_pos.parameter[0] = i;
		// parameter_block_pos.parameter[1] = new_path[i].x();
		// parameter_block_pos.parameter[2] = new_path[i].y(); 
		// parameter_block_pos.parameter[3] = new_path[i].z(); 
		parameter_block_pos.parameter[1] = vec_pose_init[i].x();
		parameter_block_pos.parameter[2] = vec_pose_init[i].y(); 
		parameter_block_pos.parameter[3] = vec_pose_init[i].z(); 
		parameter_block_time.parameter[0] = i;
		parameter_block_time.parameter[1] = vec_time_init[i].time;
		parameter_block_length.parameter[0] = i;
		parameter_block_length.parameter[1] = vec_len_cat_init[i].length;
		statesPos.push_back(parameter_block_pos);
		statesTime.push_back(parameter_block_time);
		statesLength.push_back(parameter_block_length);
		printf("[%lu]states.parameter=[%f %f %f %f %f]\n",i,parameter_block_pos.parameter[1],parameter_block_pos.parameter[2],parameter_block_pos.parameter[3],parameter_block_time.parameter[1],parameter_block_length.parameter[0]);
	}
    
	/*** Cost Function I : Equidistance constrain ***/
	for (int i = 0; i <  statesPos.size() - 1 ; ++i) {
		CostFunction* cost_function1  = new AutoDiffCostFunction<EquiDistanceFunctor, 1, 4, 4>(new EquiDistanceFunctor(w_alpha*10.0, initial_distance_states)); 
		problem.AddResidualBlock(cost_function1, NULL, statesPos[i].parameter, statesPos[i+1].parameter);
		if (i == 0)
			problem.SetParameterBlockConstant(statesPos[i].parameter);
		if (i == (statesPos.size() - 2))
			problem.SetParameterBlockConstant(statesPos[i+1].parameter);
	}

	/*** Cost Function II : Obstacles constrain ***/
	for (int i = 0; i < statesPos.size(); ++i) {
    	CostFunction* cost_function2  = new AutoDiffCostFunction<ObstacleDistanceFunctor::ObstaclesFunctor, 1, 4>(new ObstacleDistanceFunctor::ObstaclesFunctor(w_beta*10.0, distance_obstacle, nn_.kdtree, nn_.obs_points)); 
    	problem.AddResidualBlock(cost_function2, NULL, statesPos[i].parameter);
  	}

	/*** Cost Function III : Kinematic constrain ***/
	for (int i = 0; i < statesPos.size() - 2; ++i) {
		CostFunction* cost_function3  = new AutoDiffCostFunction<KinematicsFunctor, 1, 4, 4, 4>(new KinematicsFunctor(w_gamma, angle_min_traj)); 
		problem.AddResidualBlock(cost_function3, NULL, statesPos[i].parameter, statesPos[i+1].parameter, statesPos[i+2].parameter);
	}

	/*** Cost Function IV : Time constrain ***/
	for (int i = 0; i < statesTime.size(); ++i) {
		CostFunction* cost_function4  = new AutoDiffCostFunction<TimeFunctor, 1, 2>(new TimeFunctor(w_delta, vec_time_init[i].time)); 
		problem.AddResidualBlock(cost_function4, NULL, statesTime[i].parameter);
		if (i == 0)
			problem.SetParameterBlockConstant(statesTime[i].parameter);
	}

	/*** Cost Function V : Velocity constrain ***/
	for (int i = 0; i < statesTime.size() - 1; ++i) {
		CostFunction* cost_function5  = new AutoDiffCostFunction<VelocityFunctor, 1, 4, 4, 2>(new VelocityFunctor(w_epsilon, initial_velocity)); 
		problem.AddResidualBlock(cost_function5, NULL, statesPos[i].parameter, statesPos[i+1].parameter, statesTime[i+1].parameter);
	}

	/*** Cost Function VI : Acceleration constrain ***/
	for (int i = 0; i < statesTime.size() - 2; ++i) {
		CostFunction* cost_function6  = new AutoDiffCostFunction<AccelerationFunctor, 1, 4, 4, 4, 2, 2>(new AccelerationFunctor(w_zeta, initial_acceleration)); 
		problem.AddResidualBlock(cost_function6, NULL, statesPos[i].parameter, statesPos[i+1].parameter, statesPos[i+2].parameter, statesTime[i+1].parameter, statesTime[i+2].parameter);
	}

	/*** Cost Function VII : Catenary constrain  ***/
	for (int i = 0; i < statesLength.size(); ++i) {
		CostFunction* cost_function7  = new NumericDiffCostFunction<CatenaryFunctor, CENTRAL, 2, 4, 2>
										(new CatenaryFunctor(w_eta * 10.0, distance_catenary_obstacle, vec_len_cat_init[i].length, nn_.kdtree, nn_.obs_points, ugv_pos_catenary,size,nh)); 
		problem.AddResidualBlock(cost_function7, NULL, statesPos[i].parameter, statesLength[i].parameter);
		if (i == 0)
			problem.SetParameterBlockConstant(statesLength[i].parameter);
		problem.SetParameterLowerBound(statesLength[i].parameter, 1, 0.0);
		problem.SetParameterUpperBound(statesLength[i].parameter, 1, 40.0);
	}

	// /*** Cost Function VIII : Dynamic Catenary constrain  ***/
	// // for (int i = 1; i < statesLength.size(); ++i) {
	// // 	CostFunction* cost_function8  = new NumericDiffCostFunction<CatenaryFunctor, CENTRAL, 3, 5>
	// // 									(new CatenaryFunctor(w_lambda, distance_catenary_obstacle, vec_len_cat_init[i].length, nn_.kdtree, nn_.obs_points, bound_bisection_a, bound_bisection_b, ugv_pos_catenary, size)); 
	// // 	problem.AddResidualBlock(cost_function8, NULL, statesLength[i].parameter);
	// // }

	if (!continue_optimizing)
		std::cout <<  "==================================================="  << std::endl << "Optimization  started !!" << std::endl;

	start_time_opt_ = ros::Time::now();

	ceres::Solve(options, &problem, &summary);
	std::cout << summary.BriefReport() << "\n";
	final_time_opt_ = ros::Time::now();

	std::cout << std::endl <<"Optimization Proccess Completed !!!" << std::endl << "===================================================" << std::endl << "Saving Temporal data in txt file ..." << std::endl;

	vec_pose_opt.clear();
	vec_time_opt.clear();
	vec_len_cat_opt.clear();
	vec_dist_opt.clear();
	vec_vel_opt.clear();
	vec_acc_opt.clear();
	new_path.clear(); //clear the path to set the optimizer solution as path
	

  	for (size_t i = 0; i < size; i++){
		Eigen::Vector3d position_ = Eigen::Vector3d(statesPos[i].parameter[1],statesPos[i].parameter[2],statesPos[i].parameter[3]);
		vec_pose_opt.push_back(position_);
		structLengthCatenary vLC;
		vLC.length = statesLength[i].parameter[1];
		vLC.id = i;
		vec_len_cat_opt.push_back(vLC);
		vec_time_opt.push_back(statesTime[i].parameter[1]);
	}

	// mp_.clearMarkers(catenary_marker, 100, catenary_marker_pub_);
	for(size_t i = 0; i < statesPos.size(); i++){
		std::vector<geometry_msgs::Point> points_catenary_final;
		CatenarySolver cSX_;
		cSX_.setMaxNumIterations(100);
	  	cSX_.solve(ugv_pos_catenary.x, ugv_pos_catenary.y, ugv_pos_catenary.z, statesPos[i].parameter[1], statesPos[i].parameter[2], statesPos[i].parameter[3], statesLength[i].parameter[1], points_catenary_final);
		mp_.markerPoints(catenary_marker, points_catenary_final, i, size, catenary_marker_pub_);
		double _d_ = sqrt(pow(ugv_pos_catenary.x -statesPos[i].parameter[1],2) + pow(ugv_pos_catenary.y - statesPos[i].parameter[2],2) + pow(ugv_pos_catenary.z-statesPos[i].parameter[3],2));
		printf("points_catenary_final.size()=[%lu] ugv_reel_pos[%lu] = [%f %f %f] , statesPos[%lu]=[%f %f %f] , l=[%f] d=[%f]\n",points_catenary_final.size(),i,ugv_pos_catenary.x, ugv_pos_catenary.y, ugv_pos_catenary.z,i,statesPos[i].parameter[1], statesPos[i].parameter[2], statesPos[i].parameter[3],statesLength[i].parameter[1],_d_);
		points_catenary_final.clear();
	}

	mp_.getMarkerPoints(points_marker,vec_pose_opt,"points_lines");
	mp_.getMarkerLines(lines_marker,vec_pose_opt,"points_lines");
	traj_marker_pub_.publish(points_marker);
	traj_marker_pub_.publish(lines_marker);

	writeTemporalDataAfterOptimization(size);

	ros::Duration(td_).sleep();

	output_file = path+name_output_file+"_stage_"+std::to_string(scenario_number)+"_InitPos_"+std::to_string(num_pos_initial)+"_goal_"+std::to_string(num_goal)+".txt";
	ofs.open(output_file.c_str(), std::ofstream::app);
	if (write_data_for_analysis)
		getDataForOptimizerAnalysis();

	cleanVectors();		//Clear vector after optimization and previus the next iteration

	std::cout << "...Temporal data saved in txt file." << std::endl << std::endl;
	ROS_INFO("Optimizer Local Planner: Goal position successfully achieved");
	ros::Duration(20.0).sleep();
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
	for(size_t i = 0 ; i < vec_time_init.size()-1 ; i++){
		init_traj_time_ = vec_time_init[i+1].time + init_traj_time_;
		init_traj_distance_ = vec_dist_init[i].dist + init_traj_distance_;
		init_traj_vel_ = (vec_dist_init[i].dist / vec_time_init[i+1].time ) + init_traj_vel_; 
		if (init_traj_vel_max_ < (vec_dist_init[i].dist / vec_time_init[i+1].time))
			init_traj_vel_max_ = (vec_dist_init[i].dist / vec_time_init[i+1].time);
	}
	init_traj_vel_mean_ = init_traj_vel_ / ((double)vec_time_init.size()-1.0);

	//Acceleration Trajectory Initial
	for(size_t i = 0 ; i < vec_time_init.size()-2 ; i++){
		init_traj_acc_ = ( ((vec_dist_init[i].dist / vec_time_init[i+1].time ) - (vec_dist_init[i+1].dist/vec_time_init[i+2].time)) / (vec_time_init[i+1].time +vec_time_init[i+2].time) ) + init_traj_acc_; ;
		if (init_traj_acc_max_ < ( ((vec_dist_init[i].dist / vec_time_init[i+1].time )- (vec_dist_init[i+1].dist/vec_time_init[i+2].time)) / (vec_time_init[i+1].time +vec_time_init[i+2].time) ) )
			init_traj_acc_max_ = ( ((vec_dist_init[i].dist / vec_time_init[i+1].time )- (vec_dist_init[i+1].dist/vec_time_init[i+2].time)) / (vec_time_init[i+1].time +vec_time_init[i+2].time) );
	}
	init_traj_acc_mean_ = init_traj_acc_ / (double)(vec_time_init.size()-2.0);

	//Distance Obstacles Initial
	double distance_obs_init_ , distance_obs_init_min_, distance_obs_init_mean_;
	distance_obs_init_ = distance_obs_init_mean_ = 0.0;
	distance_obs_init_min_ = 1000.0;
	for(size_t i = 0 ; i < vec_pose_init.size(); i ++){
		Eigen::Vector3d p_init_ = vec_pose_init[i];
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
	for(size_t i = 0 ; i < vec_pose_init.size(); i ++){
		CatenarySolver cS_;
		v_points_catenary_init_.clear();
		cS_.setMaxNumIterations(100);
		cS_.solve(ugv_pos_catenary.x, ugv_pos_catenary.y, ugv_pos_catenary.z, vec_pose_init[i].x(), vec_pose_init[i].y(), vec_pose_init[i].z(), vec_len_cat_init[i].length, v_points_catenary_init_);
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
	for (size_t i = 0; i < vec_time_opt.size() -1 ; i++){
		opt_traj_time_ = vec_time_opt[i+1] + opt_traj_time_;
		opt_traj_distance_ = vec_dist_opt[i] + opt_traj_distance_;
		opt_traj_vel_ = vec_vel_opt[i] + opt_traj_vel_;
		if (opt_traj_vel_max_ < vec_vel_opt[i])
			opt_traj_vel_max_ = vec_vel_opt[i];
	}
	opt_traj_vel_mean_ = opt_traj_vel_ / ((double)vec_time_opt.size()-1.0);

	//Acceleration Trajectory Optimized
	for (size_t i = 0; i < vec_acc_opt.size() ; i++){
		opt_traj_acc_ = vec_acc_opt[i] + opt_traj_acc_;
		if (fabs(opt_traj_acc_max_) < fabs(vec_acc_opt[i]))
			opt_traj_acc_max_ = vec_acc_opt[i];
	}
	opt_traj_acc_mean_ = opt_traj_acc_ / (double)vec_acc_opt.size();

	//Distance Point Obstacles
	double distance_obs_opt_ , distance_obs_opt_min_, distance_obs_opt_mean_;
	distance_obs_opt_ = distance_obs_opt_mean_ = 0.0;
	distance_obs_opt_min_ = 1000.0;
	for(size_t i = 0 ; i < vec_pose_opt.size(); i ++){
		Eigen::Vector3d p_opt_ = vec_pose_opt[i];
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

	for(size_t i = 0 ; i < vec_pose_opt.size(); i ++){
		CatenarySolver cS_;
		v_points_catenary_opt_.clear();
		cS_.setMaxNumIterations(100);
		cS_.solve(ugv_pos_catenary.x, ugv_pos_catenary.y, ugv_pos_catenary.z, vec_pose_init[i].x(), vec_pose_init[i].y(), vec_pose_init[i].z(), vec_len_cat_opt[i].length, v_points_catenary_opt_);
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
				if(distance_obs_cat_opt_min_ > (p_cat_opt_- nearest_obs_p).norm() )
					distance_obs_cat_opt_min_ = (p_cat_opt_- nearest_obs_p).norm();
			}
		// printf("Obstacles Optimized: state=[%lu/%lu] vec_pose_opt.size=[%lu] v_points_catenary_opt_.size=[%lu] count_cat_p_opt = [%i]\n",i, vec_pose_opt.size(),vec_pose_opt.size(),v_points_catenary_opt_.size(),count_cat_p_opt_);
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
	double sum_dispos_ = 0.0;
	double new_vel_ = 0.0;
	double sum_difftime_ = 0.0;
	for (size_t i=0; i < _s -1; ++i){
		if ( i == 0)
			file_out_time << setprecision(6) << sum_dispos_ << ";" << vec_time_opt[i] << endl;
		double difftime_ = vec_time_opt[i+1];
		double dist_ = (vec_pose_opt[i] - vec_pose_opt[i+1]).norm();
		vec_dist_opt.push_back(dist_);
		sum_dispos_ = dist_ + sum_dispos_;
		new_vel_ = dist_ / difftime_;
		vec_vel_opt.push_back(new_vel_);
		sum_difftime_ = sum_difftime_ + difftime_;
		file_out_time << setprecision(6) << sum_dispos_ << ";" << sum_difftime_ << endl;
		file_out_velocity << setprecision(6) << sum_dispos_ << ";" << new_vel_ << endl;
		// 	file_out_difftime << setprecision(6) << sum_dispos << ";" << difftime->estimate() << endl;
	}
	double sumdis_ = 0.0;
	for (size_t i = 0; i < _s-2; i++)
	{
		double difftime1_ = statesTime[i+1].parameter[1];
		double difftime2_ = statesTime[i+2].parameter[1];

		double distance1_ = (vec_pose_opt[i+1] - vec_pose_opt[i]).norm();	
		double distance2_ = (vec_pose_opt[i+2] - vec_pose_opt[i+1]).norm();
		if (i==0)
			sumdis_ = distance1_;
		sumdis_ = sumdis_ + distance2_;
		double sumTime_ = difftime1_ + difftime2_;
		double velocity1_ = distance1_ / difftime1_;
		double velocity2_ = distance2_ / difftime2_;
		double acceleration_ = (velocity2_-velocity1_)/sumTime_;
		vec_acc_opt.push_back(acceleration_);
		file_out_acceleration << setprecision(6) << sumdis_ << ";" << acceleration_ << endl;
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
	
	double _sum_dist = 0.0;
	double _sum_time = 0.0;
	for (size_t i = 0; i < vec_time_init.size() - 1; i++){
		_sum_dist = _sum_dist + vec_dist_init[i].dist;
		_sum_time = _sum_time + vec_time_init[i+1].time;
		// if (i== 0)
		// 	file_in_time << setprecision(6) << _sum_dist << ";" << vec_time_init[i].time << endl;
		file_in_time << setprecision(6) << _sum_dist << ";" << _sum_time << endl;
		file_in_velocity  << setprecision(6) << _sum_dist << ";" << initial_velocity << endl;
		if (i > 0)
			file_in_acceleration << setprecision(6) << _sum_dist << ";" << initial_acceleration << endl;
	}
	file_in_time.close();	
	file_in_velocity.close();	
	file_in_acceleration.close();
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
    double pL = 0.0;
	double sum_distance_ = 0.0;
    for (size_t i = 0; i < _path.size()-1; i++)
    {
    	x = new_path[i+1].x() - new_path[i].x();
		y = new_path[i+1].y() - new_path[i].y();
		z = new_path[i+1].z() - new_path[i].z();
        pL= sqrt(x * x + y * y + z * z);
		sum_distance_ = sum_distance_ + pL;
		_sD.id_from = i;
		_sD.id_to = i+ 1;
		_sD.dist = pL;
		_v_sD.push_back(_sD);
		// printf("calculateDistanceVertices: dist = [%f]\n",pL);
    }
	initial_distance_states = sum_distance_ / (_path.size()-1);
}

void OptimizerLocalPlanner::getTemporalState(vector<structTime> &_time, vector<structDistance> _v_sD, double _vel)
{
	structTime _sT;
	_time.clear();
	double _dT = 0;
	// double _sum_time = 0.0;
	for (size_t i= 0 ; i < new_path.size(); i++){
		_sT.id = i;
		if (i == 0){
			_sT.time = 0.0;
			_time.push_back(_sT);
		}
		else{
			_dT = (_v_sD[i-1].dist)/_vel;
			// _sum_time = _sum_time + _dT;
			// _sT.time = _sum_time;
			_sT.time = _dT;
			_time.push_back(_sT);
		}
	// printf("getTemporalState: time = [%f]\n",_dT);
	}
}

void OptimizerLocalPlanner::setInitialLengthCatenaryAndPosUGV(std::vector <double> &_vector, auto _s)
{
	tfListener();

	_vector.clear();
	for (size_t i = 0; i < _s; i++){
		double _dist_X_c_v = (ugv_pos_catenary.x - new_path[i].x());
		double _dist_Y_c_v = (ugv_pos_catenary.y - new_path[i].y());
		double _dist_Z_c_v = (ugv_pos_catenary.z - new_path[i].z());
		
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
		int _state;
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
	int change_state = -1;

	while (i < _s){
		double _dist_X_c_v = (ugv_pos_catenary.x - new_path[i].x());
		double _dist_Y_c_v = (ugv_pos_catenary.y - new_path[i].y());
		double _dist_Z_c_v = (ugv_pos_catenary.z - new_path[i].z());
		double _dist_c_v = sqrt(_dist_X_c_v * _dist_X_c_v + _dist_Y_c_v * _dist_Y_c_v + _dist_Z_c_v * _dist_Z_c_v);
		double _length = _dist_c_v * _mF;

		if (change_state != i){
			_cat_states._state = i;
			_cat_states._initial = _length;
			_cat_states._final = -1;
			_cat_states._n_collision = -1;
			_cat_states._mf = -1;
			change_state = i;
		}

		_state_collision_z = false;
		_n_collision = 0;
		
		CatenarySolver cS_;
		_pre_points_catenary.clear();
		cS_.setMaxNumIterations(100);
  		cS_.solve(ugv_pos_catenary.x,ugv_pos_catenary.y,ugv_pos_catenary.z,new_path[i].x(),new_path[i].y(),new_path[i].z(), _length, _pre_points_catenary);

		int _n_points_cat_dis = ceil(1.5*ceil(_length)); // parameter to ignore collsion points in the begining and in the end of catenary
		if (_n_points_cat_dis < 5)
			_n_points_cat_dis = 5;

		for (size_t j = 0 ; j <_pre_points_catenary.size() ; j++){
			if (j > _n_points_cat_dis ){
				if(_pre_points_catenary[j].z < _bound_z_negative){
					ROS_ERROR("Pre Compute Catenary with z_value(%f) < z_bound(%f): IMPOSIBLE continue increasing length=[%f] for state=[%lu]",_pre_points_catenary[j].z,_bound_z_negative,_length,i);
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
		printf("preComputeLengthCatenary: Got it PRE LENGTH catenary = [state=%i , initial=%f , final=%f , n_coll=%i , mf=%f]\n",
		_v_cat_states[i]._state,_v_cat_states[i]._initial,_v_cat_states[i]._final,_v_cat_states[i]._n_collision,_v_cat_states[i]._mf);
	}
}

void OptimizerLocalPlanner::tfListener(){

	tf::StampedTransform transform;
    listener.lookupTransform("/map", "/reel_base_link", ros::Time(0), transform);

	ugv_pos_catenary.x = transform.getOrigin().x();
	ugv_pos_catenary.y = transform.getOrigin().y();
	ugv_pos_catenary.z = transform.getOrigin().z();
}

void OptimizerLocalPlanner::checkObstaclesBetweenCatenaries(std::vector <double> _vectorIN, auto _s)
{
	std::vector <double> _vectorOUT;
	int _num_points;
	double _length1,_length2;
	bool obst_between_catenaries=false;
	std::vector<geometry_msgs::Point> _pre_points_catenary1;
	std::vector<geometry_msgs::Point> _pre_points_catenary2;
	std::vector<geometry_msgs::Point> _v_factor1, _v_factor2;
	std::vector<Eigen::Vector3d> _vector_points_line;
	// bisectionCatenary bsC1, bsC2;
	CatenarySolver _cS1, _cS2;


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
		
		_pre_points_catenary1.clear();
		_cS1.setMaxNumIterations(100);
  		_cS1.solve(ugv_pos_catenary.x,ugv_pos_catenary.y,ugv_pos_catenary.z,new_path[i].x(),new_path[i].y(),new_path[i].z(), _length1, _pre_points_catenary1);
		
		_pre_points_catenary2.clear();
		_cS2.setMaxNumIterations(100);
  		_cS2.solve(ugv_pos_catenary.x,ugv_pos_catenary.y,ugv_pos_catenary.z,new_path[i+1].x(),new_path[i+1].y(),new_path[i+1].z(), _length2, _pre_points_catenary2);

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
		mp_.markerPoints(catenary_marker ,_pre_points_catenary1, i-1, _s-2, catenary_marker_pub_);
		mp_.markerPoints(catenary_marker ,_pre_points_catenary2, i,_s-2, catenary_marker_pub_);
	}
	
	printf("---------------------------------------------------------------------------------------------\n");
	for(size_t i = 0; i < _vectorOUT.size(); i++){
		structLengthCatenary l_;
		l_.id = i;
		l_.length = _vectorOUT[i];
		vec_len_cat_init.push_back(l_);
		printf("checkObstaclesBetweenCatenaries: Got it LENGTH catenary = [state=%lu , initial=%f , final=%f ]\n",i,_vectorIN[i],_vectorOUT[i]);
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
