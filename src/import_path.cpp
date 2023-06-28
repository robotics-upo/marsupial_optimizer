#include "misc/import_path.hpp"

ImportPath::ImportPath()
{}

ImportPath::ImportPath(const std::string &path_and_name_file_, upo_actions::ExecutePathGoal &g_)
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