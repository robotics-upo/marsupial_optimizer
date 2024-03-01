#include "misc/manage_computed_path.hpp"

ManagePath::ManagePath()
{}

ManagePath::ManagePath(const std::string &path_and_name_file_, upo_actions::ExecutePathGoal &g_)
{
  YAML::Node file = YAML::LoadFile(path_and_name_file_);

  trajectory.points.clear(); 

  trajectory_msgs::MultiDOFJointTrajectoryPoint t_;
  t_.transforms.resize(2);
  t_.velocities.resize(2);
  t_.accelerations.resize(2);

  tether_length_vector.clear();

  int size_ = (file["marsupial_ugv"]["size"].as<int>()) ; 
  std::string ugv_pos_data, uav_pos_data, tether_data;
  double ugv_pos_x, ugv_pos_y, ugv_pos_z, ugv_rot_x, ugv_rot_y, ugv_rot_z, ugv_rot_w;
  double uav_pos_x, uav_pos_y, uav_pos_z, uav_rot_x, uav_rot_y, uav_rot_z, uav_rot_w;
  for (int i = 0; i < size_; i++) {
    // It begin in 1 because first point is given as initial point.
    ugv_pos_data = "poses" + std::to_string(i);
    uav_pos_data = "poses" + std::to_string(i);
    tether_data = "length" + std::to_string(i);
    try {
	init_uav_pose.orientation.z =
	  file["marsupial_uav"][uav_pos_data]["pose"]["orientation"]["z"].as<double>();
	init_uav_pose.orientation.w =
	  file["marsupial_uav"][uav_pos_data]["pose"]["orientation"]["w"].as<double>();
	if (i==0) {
	    init_ugv_pose.position.x = file["marsupial_ugv"][ugv_pos_data]["pose"]["position"]["x"].as<double>();
	    init_ugv_pose.position.y = file["marsupial_ugv"][ugv_pos_data]["pose"]["position"]["y"].as<double>();
	    init_ugv_pose.position.z = file["marsupial_ugv"][ugv_pos_data]["pose"]["position"]["z"].as<double>();
	    init_ugv_pose.orientation.x = file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["x"].as<double>();
	    init_ugv_pose.orientation.y = file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["y"].as<double>();
	    init_ugv_pose.orientation.z = file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["z"].as<double>();
	    init_ugv_pose.orientation.w = file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["w"].as<double>();
	    init_uav_pose.position.x = (file["marsupial_uav"][uav_pos_data]["pose"]["position"]["x"].as<double>());
	    init_uav_pose.position.y = (file["marsupial_uav"][uav_pos_data]["pose"]["position"]["y"].as<double>());	    
        init_uav_pose.position.z = (file["marsupial_uav"][uav_pos_data]["pose"]["position"]["z"].as<double>());
	    init_uav_pose.orientation.x =
	      file["marsupial_uav"][uav_pos_data]["pose"]["orientation"]["x"].as<double>();
	    init_uav_pose.orientation.y =
	      file["marsupial_uav"][uav_pos_data]["pose"]["orientation"]["y"].as<double>();
	    init_uav_pose.orientation.z =
	      file["marsupial_uav"][uav_pos_data]["pose"]["orientation"]["z"].as<double>();
	    init_uav_pose.orientation.w =
	      file["marsupial_uav"][uav_pos_data]["pose"]["orientation"]["w"].as<double>();
	}
           
	ugv_pos_x = file["marsupial_ugv"][ugv_pos_data]["pose"]["position"]["x"].as<double>();
	ugv_pos_y = file["marsupial_ugv"][ugv_pos_data]["pose"]["position"]["y"].as<double>();
	ugv_pos_z = file["marsupial_ugv"][ugv_pos_data]["pose"]["position"]["z"].as<double>();
	ugv_rot_x = file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["x"].as<double>();
	ugv_rot_y = file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["y"].as<double>();
	ugv_rot_z = file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["z"].as<double>();
	ugv_rot_w = file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["w"].as<double>();
	uav_pos_x = file["marsupial_uav"][uav_pos_data]["pose"]["position"]["x"].as<double>();
	uav_pos_y = file["marsupial_uav"][uav_pos_data]["pose"]["position"]["y"].as<double>();
	uav_pos_z = file["marsupial_uav"][uav_pos_data]["pose"]["position"]["z"].as<double>();
	uav_rot_x = file["marsupial_uav"][uav_pos_data]["pose"]["orientation"]["x"].as<double>();
	uav_rot_y = file["marsupial_uav"][uav_pos_data]["pose"]["orientation"]["y"].as<double>();
	uav_rot_z = file["marsupial_uav"][uav_pos_data]["pose"]["orientation"]["z"].as<double>();
	uav_rot_w = file["marsupial_uav"][uav_pos_data]["pose"]["orientation"]["w"].as<double>();
	
	t_.transforms[0].translation.x = ugv_pos_x;
	t_.transforms[0].translation.y = ugv_pos_y;
	t_.transforms[0].translation.z = ugv_pos_z;
	t_.transforms[0].rotation.x = ugv_rot_x;
	t_.transforms[0].rotation.y = ugv_rot_y;
	t_.transforms[0].rotation.z = ugv_rot_z;
	t_.transforms[0].rotation.w = ugv_rot_w;
	t_.velocities[0].linear.x = 0.0;
	t_.velocities[0].linear.y = 0.0;
	t_.velocities[0].linear.z = 0.0;
	t_.accelerations[0].linear.x = 0.0;
	t_.accelerations[0].linear.y = 0.0;
	t_.accelerations[0].linear.z = 0.0;
	t_.transforms[1].translation.x = uav_pos_x;
	t_.transforms[1].translation.y = uav_pos_y;
	t_.transforms[1].translation.z = uav_pos_z;
	t_.transforms[1].rotation.x = uav_rot_x;
	t_.transforms[1].rotation.y = uav_rot_y;
	t_.transforms[1].rotation.z = uav_rot_z;
	t_.transforms[1].rotation.w = uav_rot_w;
	t_.velocities[1].linear.x = 0.0;
	t_.velocities[1].linear.y = 0.0;
	t_.velocities[1].linear.z = 0.0;
	t_.accelerations[1].linear.x = 0.0;
	t_.accelerations[1].linear.y = 0.0;
	t_.accelerations[1].linear.z = 0.0;
	t_.time_from_start = ros::Duration(0.5);
	trajectory.points.push_back(t_);

	float length = 2.0;
	try {
	  length = file["tether"][tether_data]["length"].as<double>();
	} catch (std::exception &e) {}
	tether_length_vector.push_back(length); // TODO calculate the distance bw UAV and UGV
    } catch(std::exception &e) {
      ROS_INFO("Skipping waypoint %d", i);
    }
  }
  std::cout << "YAML FILE readed. YAML FILE NAME: " << path_and_name_file_ << std::endl;
  std::cout << "Number of points: " << trajectory.points.size() << std::endl;
  
    upo_actions::ExecutePathGoal goal_action;
    g_.path = trajectory;
    for(int i= 0; i<tether_length_vector.size() ; i++){
        g_.length_catenary.push_back(tether_length_vector[i]);
    }
}

void ManagePath::exportOptimizedPath(vector<geometry_msgs::Vector3> &v_ugv_, vector<geometry_msgs::Vector3> &v_uav_, 
									 vector<geometry_msgs::Quaternion> &v_r_ugv_, vector<geometry_msgs::Quaternion> &v_r_uav_, vector<float> &v_l_,
									 string path_mission_file_)
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
	printf("vec_pose_ugv_opt.size()=%lu , vec_pose_uav_opt.size()=%lu , vec_len_cat_opt.size()=%lu\n",v_ugv_.size(),v_uav_.size(),v_l_.size());
	for (int i=0 ; i < v_ugv_.size()-1; i++){
		d_to_interp_ = 0.2;
		interpol_ = false;
		d1_ = sqrt ( pow(v_ugv_[i+1].x - v_ugv_[i].x,2) + pow(v_ugv_[i+1].y - v_ugv_[i].y,2) + pow(v_ugv_[i+1].z - v_ugv_[i].z,2) );
		d2_ = sqrt ( pow(v_uav_[i+1].x - v_uav_[i].x,2) + pow(v_uav_[i+1].y - v_uav_[i].y,2) + pow(v_uav_[i+1].z - v_uav_[i].z,2) );
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
				p_int_ugv_.x = v_ugv_[i].x + (j+1)*(v_ugv_[i+1].x - v_ugv_[i].x)/(r_+1);
				p_int_ugv_.y = v_ugv_[i].y + (j+1)*(v_ugv_[i+1].y - v_ugv_[i].y)/(r_+1);
				p_int_ugv_.z = v_ugv_[i].z + (j+1)*(v_ugv_[i+1].z - v_ugv_[i].z)/(r_+1);
				p_int_uav_.x = v_uav_[i].x + (j+1)*(v_uav_[i+1].x - v_uav_[i].x)/(r_+1);
				p_int_uav_.y = v_uav_[i].y + (j+1)*(v_uav_[i+1].y - v_uav_[i].y)/(r_+1);
				p_int_uav_.z = v_uav_[i].z + (j+1)*(v_uav_[i+1].z - v_uav_[i].z)/(r_+1);
				cat_inter_ = v_l_[i] + (j+1)*(v_l_[i+1] - v_l_[i])/(r_+1);
				v_interp_pose_ugv.push_back(p_int_ugv_);
				v_interp_rot_ugv.push_back(v_r_ugv_[i]);
				v_interp_pose_uav.push_back(p_int_uav_);
				v_interp_rot_uav.push_back(v_r_uav_[i]);
				vec_interp_len_cat.push_back(cat_inter_);
			}
		}else{
			v_interp_pose_ugv.push_back(v_ugv_[i]);
			v_interp_rot_ugv.push_back(v_r_ugv_[i]);
			v_interp_pose_uav.push_back(v_uav_[i]);
			v_interp_rot_uav.push_back(v_r_uav_[i]);
			vec_interp_len_cat.push_back(v_l_[i]);
		}
	}
	v_ugv_.clear();
	v_r_ugv_.clear();
	v_uav_.clear();
	v_r_uav_.clear();
	v_l_.clear();
	v_ugv_ = v_interp_pose_ugv;
	v_r_ugv_ = v_interp_rot_ugv;
	v_uav_ = v_interp_pose_uav;
	v_r_uav_ = v_interp_rot_uav;
	v_l_ = vec_interp_len_cat;
	printf("v_ugv_.size()=%lu , v_uav_.size()=%lu vec_interp_len_cat=%lu\n",
	v_interp_pose_ugv.size(),v_interp_pose_ugv.size(),vec_interp_len_cat.size());

    // Root of our file
    YAML::Node root;

    // Populate emitter
    YAML::Emitter emitter;

    // // Create a node listing some values
    
    // // We now will write our values under root["MyNode"]
    YAML::Node node = YAML::Node(YAML::NodeType::Map);
    // YAML::Node node;
    // Write some values
    int size_ = v_ugv_.size();

    node["header"] = "marsupial_ugv";
    node["seq"] = 1;
    node["stamp"] = hour_+min_+sec_;
    node["frame_id"] = "ugv";
    node["size"] = size_;
    
    for(int i=0 ; i < v_ugv_.size(); i++){
		node["poses"+to_string(i)]["header"] = "ugv"+to_string(i);
        node["poses"+to_string(i)]["seq"] = i;
        node["poses"+to_string(i)]["frame_id"] = "ugv";  
        node["poses"+to_string(i)]["pose"]["position"]["x"] = v_ugv_[i].x;
        node["poses"+to_string(i)]["pose"]["position"]["y"] = v_ugv_[i].y;
        node["poses"+to_string(i)]["pose"]["position"]["z"] = v_ugv_[i].z;
        node["poses"+to_string(i)]["pose"]["orientation"]["x"] = v_r_ugv_[i].x;
        node["poses"+to_string(i)]["pose"]["orientation"]["y"] = v_r_ugv_[i].y;
        node["poses"+to_string(i)]["pose"]["orientation"]["z"] = v_r_ugv_[i].z;
        node["poses"+to_string(i)]["pose"]["orientation"]["w"] = v_r_ugv_[i].w;
    }

	root["marsupial_ugv"] = node;

    // // Create a node listing some values
    node = YAML::Node(YAML::NodeType::Map);
	size_ = v_uav_.size();
    // YAML::Node node;
    // Write some values
    node["header"] = "marsupial_uav";
    node["seq"] = 1;
    node["stamp"] = hour_+min_+sec_ ;
    node["frame_id"] = "base_link_uav";
    node["size"] = size_;
    for(int i=0 ; i < v_uav_.size(); i++){
        node["poses"+to_string(i)]["header"] = "uav"+to_string(i);
        node["poses"+to_string(i)]["seq"] = i;
        node["poses"+to_string(i)]["frame_id"] = "uav";  
        node["poses"+to_string(i)]["pose"]["position"]["x"] = v_uav_[i].x;
        node["poses"+to_string(i)]["pose"]["position"]["y"] = v_uav_[i].y;
        node["poses"+to_string(i)]["pose"]["position"]["z"] = v_uav_[i].z;
        node["poses"+to_string(i)]["pose"]["orientation"]["x"] = v_r_uav_[i].x;
        node["poses"+to_string(i)]["pose"]["orientation"]["y"] = v_r_uav_[i].y;
        node["poses"+to_string(i)]["pose"]["orientation"]["z"] = v_r_uav_[i].z;
        node["poses"+to_string(i)]["pose"]["orientation"]["w"] = v_r_uav_[i].w;
    }

	root["marsupial_uav"] = node;

	// // Create a node listing some values
    node = YAML::Node(YAML::NodeType::Map);
	size_ = v_l_.size();
    // YAML::Node node;
    // Write some values
    node["header"] = "tether";
    node["seq"] = 1;
    node["stamp"] = hour_+min_+sec_ ;
    node["frame_id"] = "frame_tether";
    node["size"] = size_;
    for(int i=0 ; i < v_l_.size(); i++){
        node["length"+to_string(i)]["header"] = "tether"+to_string(i);
        node["length"+to_string(i)]["seq"] = i;
        node["length"+to_string(i)]["frame_id"] = "tether_length";  
        node["length"+to_string(i)]["length"] = v_l_[i];
    }
	root["tether"] = node;


    // Populate emitter
    emitter << root;

    // Write to file
    std::ofstream fout;
	fout.open(path_mission_file_+"optimized_path_"+year_+"_"+month_+"_"+day_+"_"+hour_+min_+sec_ +".yaml", std::ofstream::app);
    fout << emitter.c_str();

	std::cout << "Saved Path Optimized" << std::endl << std::endl;
}

void ManagePath::publishOptimizedTraj(vector<geometry_msgs::Vector3> v_ugv_, vector<geometry_msgs::Vector3> v_uav_, 
									 vector<geometry_msgs::Quaternion> v_r_ugv_, vector<geometry_msgs::Quaternion> v_r_uav_, 
									 vector<float> v_l_, vector<double> v_t_, marsupial_optimizer::marsupial_trajectory_optimized &msg_)
{
    ROS_INFO("Executing Trajectories : Tf modification");

	trajectory_msgs::MultiDOFJointTrajectoryPoint t_;
	trajectory_msgs::MultiDOFJointTrajectory trajectory_;
	std::vector<float> l_cat_;

    trajectory_.points.clear();

	t_.transforms.resize(2);
	t_.velocities.resize(2);
	t_.accelerations.resize(2);

	for (size_t i = 0; i <  v_ugv_.size() ; i++){
		//For UGV
		t_.transforms[0].translation.x = v_ugv_[i].x;
		t_.transforms[0].translation.y = v_ugv_[i].y;
		t_.transforms[0].translation.z = v_ugv_[i].z;
		t_.transforms[0].rotation.x = v_r_ugv_[i].x;
		t_.transforms[0].rotation.y = v_r_ugv_[i].y;
		t_.transforms[0].rotation.z = v_r_ugv_[i].z;
		t_.transforms[0].rotation.w = v_r_ugv_[i].w;
		if (i==0){
			t_.velocities[0].linear.x = 0.0;
			t_.velocities[0].linear.y = 0.0;
			t_.velocities[0].linear.z = 0.0;
		}else{ 
			t_.velocities[0].linear.x = (v_ugv_[i].x - v_ugv_[i-1].x)/v_t_[i];
			t_.velocities[0].linear.y = (v_ugv_[i].y - v_ugv_[i-1].y)/v_t_[i];
			t_.velocities[0].linear.z = (v_ugv_[i].z - v_ugv_[i-1].z)/v_t_[i];
		}
		if (i==0 || i==v_ugv_.size()-1){
			t_.accelerations[0].linear.x = 0.0;
			t_.accelerations[0].linear.y = 0.0;
			t_.accelerations[0].linear.z = 0.0;
		}
		else{
			t_.accelerations[0].linear.x= ((v_ugv_[i+1].x - v_ugv_[i].x)/v_t_[i+1])-((v_ugv_[i].x - v_ugv_[i-1].x)/v_t_[i]);
			t_.accelerations[0].linear.y= ((v_ugv_[i+1].y - v_ugv_[i].y)/v_t_[i+1])-((v_ugv_[i].y - v_ugv_[i-1].y)/v_t_[i]);
			t_.accelerations[0].linear.z= ((v_ugv_[i+1].z - v_ugv_[i].z)/v_t_[i+1])-((v_ugv_[i].z - v_ugv_[i-1].z)/v_t_[i]);
		}
		//For UAV
		t_.transforms[1].translation.x = v_uav_[i].x;
		t_.transforms[1].translation.y = v_uav_[i].y;
		t_.transforms[1].translation.z = v_uav_[i].z;
		t_.transforms[1].rotation.x = v_r_uav_[i].x;
		t_.transforms[1].rotation.y = v_r_uav_[i].y;
		t_.transforms[1].rotation.z = v_r_uav_[i].z;
		t_.transforms[1].rotation.w = v_r_uav_[i].w;
		if (i==0){
			t_.velocities[1].linear.x = 0.0;
			t_.velocities[1].linear.y = 0.0;
			t_.velocities[1].linear.z = 0.0;
		}else{  
			t_.velocities[1].linear.x = (v_uav_[i].x - v_uav_[i-1].x)/v_t_[i];
			t_.velocities[1].linear.y = (v_uav_[i].y - v_uav_[i-1].y)/v_t_[i];
			t_.velocities[1].linear.z = (v_uav_[i].z - v_uav_[i-1].z)/v_t_[i];
		}
		if (i==0 || i==v_ugv_.size()-1){
			t_.accelerations[1].linear.x = 0.0;
			t_.accelerations[1].linear.y = 0.0;
			t_.accelerations[1].linear.z = 0.0;
		}else{
			t_.accelerations[1].linear.x= ((v_uav_[i+1].x - v_uav_[i].x)/v_t_[i+1])-((v_uav_[i].x - v_uav_[i-1].x)/v_t_[i]);
			t_.accelerations[1].linear.y= ((v_uav_[i+1].y - v_uav_[i].y)/v_t_[i+1])-((v_uav_[i].y - v_uav_[i-1].y)/v_t_[i]);
			t_.accelerations[1].linear.z= ((v_uav_[i+1].z - v_uav_[i].z)/v_t_[i+1])-((v_uav_[i].z - v_uav_[i-1].z)/v_t_[i]);
		}
		// t_.time_from_start = v_t_[i].x;
		trajectory_.header.stamp = ros::Time::now();
		trajectory_.points.push_back(t_);
		l_cat_.push_back(v_l_[i]);
	}

	msg_.trajectory = trajectory_;
	msg_.length_catenary = l_cat_;
}