#include <ctime>
#include <string>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// #include "catenary_checker/bisection_catenary_3D.h"

using namespace std;

class graphOptimizedTrajectory{

    public:
        graphOptimizedTrajectory(ros::NodeHandlePtr nh, ros::NodeHandle pnh);
        void readWaypoints(const std::string &file_yaml_);
        void markerPoints();
        void goalPointMarker();

        double offset_map_dll_x ,offset_map_dll_y ,offset_map_dll_z;
        string file_yaml;
        ros::Publisher traj_uav_pub_, traj_ugv_pub_, catenary_length_pub_, goal_point_pub_;
        ros::Publisher traj_lines_ugv_pub_, traj_lines_uav_pub_, catenary_marker_pub_;
        std::vector<float> tether_length_vector;
        // bisectionCatenary BisCat;
        trajectory_msgs::MultiDOFJointTrajectory trajectory;

    private:

    protected:
};

graphOptimizedTrajectory::graphOptimizedTrajectory(ros::NodeHandlePtr nh, ros::NodeHandle pnh){

    printf("\n\tInitialazing Check_Mission_NODE !!\n");
    
    nh.reset(new ros::NodeHandle("~"));
    pnh.param("file_yaml", file_yaml, (std::string) "~/");

    traj_ugv_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("trajectory_ugv", 100);
    traj_uav_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("trajectory_uav", 100);
    traj_lines_ugv_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("trajectory_lines_ugv", 100);
    traj_lines_uav_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("trajectory_lines_uav", 100);
    catenary_marker_pub_= pnh.advertise<visualization_msgs::MarkerArray>("trajectory_catenary", 100);
    goal_point_pub_= pnh.advertise<visualization_msgs::Marker>("/random_planner_node/goal_point", 100);
}

void graphOptimizedTrajectory::readWaypoints(const std::string &file_yaml_)
{
  YAML::Node file = YAML::LoadFile(file_yaml_);

  trajectory.points.clear(); 

  trajectory_msgs::MultiDOFJointTrajectoryPoint traj_marsupial_;

  traj_marsupial_.transforms.resize(2);
  traj_marsupial_.velocities.resize(2);
  traj_marsupial_.accelerations.resize(2);

  int size_ = (file["size"].as<int>()) ; 
  std::string ugv_pos_data, uav_pos_data, tether_data;
  double ugv_pos_x, ugv_pos_y, ugv_pos_z, ugv_rot_x, ugv_rot_y, ugv_rot_z, ugv_rot_w;
  double uav_pos_x, uav_pos_y, uav_pos_z, uav_rot_x, uav_rot_y, uav_rot_z, uav_rot_w;
  printf("offset_map_dll=[%f %f %f]\n",offset_map_dll_x,offset_map_dll_y,offset_map_dll_z);
  for (int i = 0; i < size_; i++) {
    // It begin in 1 because first point is given as initial point.
    ugv_pos_data = "poses" + std::to_string(i);
    uav_pos_data = "poses" + std::to_string(i);
    tether_data = "length" + std::to_string(i);
    try {    
        ugv_pos_x = file["marsupial_ugv"][ugv_pos_data]["pose"]["position"]["x"].as<double>() + offset_map_dll_x;
        ugv_pos_y = file["marsupial_ugv"][ugv_pos_data]["pose"]["position"]["y"].as<double>() + offset_map_dll_y;
        ugv_pos_z = file["marsupial_ugv"][ugv_pos_data]["pose"]["position"]["z"].as<double>() + offset_map_dll_z;
        ugv_rot_x = file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["x"].as<double>();
        ugv_rot_y = file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["y"].as<double>();
        ugv_rot_z = file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["z"].as<double>();
        ugv_rot_w = file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["w"].as<double>();
        uav_pos_x = file["marsupial_uav"][uav_pos_data]["pose"]["position"]["x"].as<double>() + offset_map_dll_x;
        uav_pos_y = file["marsupial_uav"][uav_pos_data]["pose"]["position"]["y"].as<double>() + offset_map_dll_y;
        uav_pos_z = file["marsupial_uav"][uav_pos_data]["pose"]["position"]["z"].as<double>() + offset_map_dll_z;
        uav_rot_x = file["marsupial_uav"][uav_pos_data]["pose"]["orientation"]["x"].as<double>();
        uav_rot_y = file["marsupial_uav"][uav_pos_data]["pose"]["orientation"]["y"].as<double>();
        uav_rot_z = file["marsupial_uav"][uav_pos_data]["pose"]["orientation"]["z"].as<double>();
        uav_rot_w = file["marsupial_uav"][uav_pos_data]["pose"]["orientation"]["w"].as<double>();
	
        traj_marsupial_.transforms[0].translation.x = ugv_pos_x;
        traj_marsupial_.transforms[0].translation.y = ugv_pos_y;
        traj_marsupial_.transforms[0].translation.z = ugv_pos_z;
        traj_marsupial_.transforms[0].rotation.x = ugv_rot_x;
        traj_marsupial_.transforms[0].rotation.y = ugv_rot_y;
        traj_marsupial_.transforms[0].rotation.z = ugv_rot_z;
        traj_marsupial_.transforms[0].rotation.w = ugv_rot_w;
        traj_marsupial_.velocities[0].linear.x = 0.0;
        traj_marsupial_.velocities[0].linear.y = 0.0;
        traj_marsupial_.velocities[0].linear.z = 0.0;
        traj_marsupial_.accelerations[0].linear.x = 0.0;
        traj_marsupial_.accelerations[0].linear.y = 0.0;
        traj_marsupial_.accelerations[0].linear.z = 0.0;
        traj_marsupial_.transforms[1].translation.x = uav_pos_x;
        traj_marsupial_.transforms[1].translation.y = uav_pos_y;
        traj_marsupial_.transforms[1].translation.z = uav_pos_z;
        traj_marsupial_.transforms[1].rotation.x = uav_rot_x;
        traj_marsupial_.transforms[1].rotation.y = uav_rot_y;
        traj_marsupial_.transforms[1].rotation.z = uav_rot_z;
        traj_marsupial_.transforms[1].rotation.w = uav_rot_w;
        traj_marsupial_.velocities[1].linear.x = 0.0;
        traj_marsupial_.velocities[1].linear.y = 0.0;
        traj_marsupial_.velocities[1].linear.z = 0.0;
        traj_marsupial_.accelerations[1].linear.x = 0.0;
        traj_marsupial_.accelerations[1].linear.y = 0.0;
        traj_marsupial_.accelerations[1].linear.z = 0.0;
        traj_marsupial_.time_from_start = ros::Duration(0.5);
        trajectory.points.push_back(traj_marsupial_);

	    float length = 2.0;
        try {
        length = file["tether"][tether_data]["length"].as<double>();
        } 
        catch (std::exception &e) {}

        tether_length_vector.push_back(length); // TODO calculate the distance bw UAV and UGV

    }catch(std::exception &e) {
      ROS_INFO("Skipping waypoint %d. Content: %s", i, e.what());
    }
  }
  std::cout << "YAML FILE readed. YAML FILE NAME: " << file_yaml_ << std::endl;
  std::cout << "Number of points: " << trajectory.points.size() << std::endl;
}

void graphOptimizedTrajectory::markerPoints()
{
  visualization_msgs::MarkerArray _marker_ugv, _marker_uav ,_lines_ugv, _lines_uav, _cat_marker; 
  trajectory_msgs::MultiDOFJointTrajectory _traj = trajectory;
	geometry_msgs::Point _p1, _p2; 

  _marker_ugv.markers.resize(_traj.points.size());
  _marker_uav.markers.resize(_traj.points.size());
  _lines_ugv.markers.resize(_traj.points.size());
  _lines_uav.markers.resize(_traj.points.size());

  for (size_t i = 0; i < _traj.points.size(); ++i){
    //For UGV
    _marker_ugv.markers[i].header.frame_id = "world";
    _marker_ugv.markers[i].header.stamp = ros::Time::now();
    _marker_ugv.markers[i].ns = "traj_ugv";
    _marker_ugv.markers[i].id = i+1;
    _marker_ugv.markers[i].action = visualization_msgs::Marker::ADD;
    if (i % 5 == 0)
      _marker_ugv.markers[i].type = visualization_msgs::Marker::CUBE;
    else
      _marker_ugv.markers[i].type = visualization_msgs::Marker::SPHERE;
    _marker_ugv.markers[i].lifetime = ros::Duration(0);
    _marker_ugv.markers[i].pose.position.x = _traj.points.at(i).transforms[0].translation.x;
    _marker_ugv.markers[i].pose.position.y = _traj.points.at(i).transforms[0].translation.y; 
    _marker_ugv.markers[i].pose.position.z = _traj.points.at(i).transforms[0].translation.z;
    _marker_ugv.markers[i].pose.orientation.x = 0.0;
    _marker_ugv.markers[i].pose.orientation.y = 0.0;
    _marker_ugv.markers[i].pose.orientation.z = 0.0;
    _marker_ugv.markers[i].pose.orientation.w = 1.0;
    _marker_ugv.markers[i].scale.x = 0.4;
    _marker_ugv.markers[i].scale.y = 0.4;
    _marker_ugv.markers[i].scale.z = 0.4;
    _marker_ugv.markers[i].color.a = 1.0;
    _marker_ugv.markers[i].color.r = 0.1;
    _marker_ugv.markers[i].color.g = 1.0;
    _marker_ugv.markers[i].color.b = 0.1;
    //FOR UAV
    _marker_uav.markers[i].header.frame_id = "world";
    _marker_uav.markers[i].header.stamp = ros::Time::now();
    _marker_uav.markers[i].ns = "traj_uav";
    _marker_uav.markers[i].id = i+1;
    _marker_uav.markers[i].action = visualization_msgs::Marker::ADD;
    if (i % 5 == 0)
      _marker_uav.markers[i].type = visualization_msgs::Marker::CUBE;
    else
      _marker_uav.markers[i].type = visualization_msgs::Marker::SPHERE;
    _marker_uav.markers[i].lifetime = ros::Duration(0);
    _marker_uav.markers[i].pose.position.x = _traj.points.at(i).transforms[1].translation.x;
    _marker_uav.markers[i].pose.position.y = _traj.points.at(i).transforms[1].translation.y; 
    _marker_uav.markers[i].pose.position.z = _traj.points.at(i).transforms[1].translation.z;
    _marker_uav.markers[i].pose.orientation.x = 0.0;
    _marker_uav.markers[i].pose.orientation.y = 0.0;
    _marker_uav.markers[i].pose.orientation.z = 0.0;
    _marker_uav.markers[i].pose.orientation.w = 1.0;
    _marker_uav.markers[i].scale.x = 0.4;
    _marker_uav.markers[i].scale.y = 0.4;
    _marker_uav.markers[i].scale.z = 0.4;
    _marker_uav.markers[i].color.a = 1.0;
    _marker_uav.markers[i].color.r = 0.1;
    _marker_uav.markers[i].color.g = 0.1;
    _marker_uav.markers[i].color.b = 1.0;

    if (i > 0) {
      _p1.x = _traj.points.at(i-1).transforms[0].translation.x;
      _p1.y = _traj.points.at(i-1).transforms[0].translation.y;
      _p1.z = _traj.points.at(i-1).transforms[0].translation.z;
      _p2.x = _traj.points.at(i).transforms[0].translation.x;
      _p2.y = _traj.points.at(i).transforms[0].translation.y;
      _p2.z = _traj.points.at(i).transforms[0].translation.z;
      _lines_ugv.markers[i].header.frame_id = "world";
      _lines_ugv.markers[i].header.stamp = ros::Time::now();
      _lines_ugv.markers[i].ns = "lines_uav";
      _lines_ugv.markers[i].id = i-1;
      _lines_ugv.markers[i].action = visualization_msgs::Marker::ADD;
      _lines_ugv.markers[i].type = visualization_msgs::Marker::LINE_STRIP;
      _lines_ugv.markers[i].lifetime = ros::Duration(0);
      _lines_ugv.markers[i].points.push_back(_p1);
      _lines_ugv.markers[i].points.push_back(_p2);
      _lines_ugv.markers[i].pose.orientation.x = 0.0;
      _lines_ugv.markers[i].pose.orientation.y = 0.0;
      _lines_ugv.markers[i].pose.orientation.z = 0.0;
      _lines_ugv.markers[i].pose.orientation.w = 1.0;
      _lines_ugv.markers[i].scale.x = 0.1;
      _lines_ugv.markers[i].color.a = 1.0;
      _lines_ugv.markers[i].color.r = 0.1;
      _lines_ugv.markers[i].color.g = 1.0;
      _lines_ugv.markers[i].color.b = 0.1;
      _p1.x = _traj.points.at(i-1).transforms[1].translation.x;
      _p1.y = _traj.points.at(i-1).transforms[1].translation.y;
      _p1.z = _traj.points.at(i-1).transforms[1].translation.z;
      _p2.x = _traj.points.at(i).transforms[1].translation.x;
      _p2.y = _traj.points.at(i).transforms[1].translation.y;
      _p2.z = _traj.points.at(i).transforms[1].translation.z;
      _lines_uav.markers[i].header.frame_id = "world";
      _lines_uav.markers[i].header.stamp = ros::Time::now();
      _lines_uav.markers[i].ns = "lines_uav";
      _lines_uav.markers[i].id = i-1;
      _lines_uav.markers[i].action = visualization_msgs::Marker::ADD;
      _lines_uav.markers[i].type = visualization_msgs::Marker::LINE_STRIP;
      _lines_uav.markers[i].lifetime = ros::Duration(0);
      _lines_uav.markers[i].points.push_back(_p1);
      _lines_uav.markers[i].points.push_back(_p2);
      _lines_uav.markers[i].pose.orientation.x = 0.0;
      _lines_uav.markers[i].pose.orientation.y = 0.0;
      _lines_uav.markers[i].pose.orientation.z = 0.0;
      _lines_uav.markers[i].pose.orientation.w = 1.0;
      _lines_uav.markers[i].scale.x = 0.1;
      _lines_uav.markers[i].color.a = 1.0;
      _lines_uav.markers[i].color.r = 0.1;
      _lines_uav.markers[i].color.g = 0.1;
      _lines_uav.markers[i].color.b = 1.0;
    }
  }	

  traj_ugv_pub_.publish(_marker_ugv);
  traj_uav_pub_.publish(_marker_uav);
  traj_lines_ugv_pub_.publish(_lines_ugv);
  traj_lines_uav_pub_.publish(_lines_uav);

  std::vector<geometry_msgs::Point> points_catenary_final;
//   for (size_t i = 0; i < _traj.points.size(); ++i){
//     points_catenary_final.clear();
//     BisCat.configBisection(tether_length_vector[i], 
// 			   _marker_ugv.markers[i].pose.position.x,
// 			   _marker_ugv.markers[i].pose.position.y,
// 			   _marker_ugv.markers[i].pose.position.z + 0.4, 
// 			   _marker_uav.markers[i].pose.position.x,
// 			   _marker_uav.markers[i].pose.position.y,
// 			   _marker_uav.markers[i].pose.position.z);
//     BisCat.getPointCatenary3D(points_catenary_final);

//     _cat_marker.markers.resize(points_catenary_final.size());
            
//     for (size_t j = 0; j < points_catenary_final.size(); ++j){
//       // double c_color1 = ((double)i / (double)points_catenary_final.size());
// 			// double c_color2 = ((double)i / (double)points_catenary_final.size());
//       _cat_marker.markers[j].header.frame_id = "world";
//       _cat_marker.markers[j].header.stamp = ros::Time::now();
//       _cat_marker.markers[j].ns = "cat_marker";
//       _cat_marker.markers[j].id = i*1000+j;
//       _cat_marker.markers[j].action = visualization_msgs::Marker::ADD;
//       if (j % 5 == 0 )
// 	_cat_marker.markers[j].type = visualization_msgs::Marker::CUBE;
//       else
// 	_cat_marker.markers[j].type = visualization_msgs::Marker::SPHERE;
//       _cat_marker.markers[j].lifetime = ros::Duration(0);
//       _cat_marker.markers[j].pose.position.x = points_catenary_final[j].x; 
//       _cat_marker.markers[j].pose.position.y = points_catenary_final[j].y; 
//       _cat_marker.markers[j].pose.position.z = points_catenary_final[j].z;

//       _cat_marker.markers[j].pose.orientation.x = 0.0;
//       _cat_marker.markers[j].pose.orientation.y = 0.0;
//       _cat_marker.markers[j].pose.orientation.z = 0.0;
//       _cat_marker.markers[j].pose.orientation.w = 1.0;
//       _cat_marker.markers[j].scale.x = 0.1;
//       _cat_marker.markers[j].scale.y = 0.1;
//       _cat_marker.markers[j].scale.z = 0.1;
//       _cat_marker.markers[j].color.a = 1.0;
//       _cat_marker.markers[j].color.r = 1.0 - i/_traj.points.size();
//       _cat_marker.markers[j].color.g = i/_traj.points.size();
//       _cat_marker.markers[j].color.b = i/_traj.points.size();
//       // _cat_marker.markers[i].color.r = 1.0 - c_color1;
// 			// _cat_marker.markers[i].color.g = c_color2;
// 			// _cat_marker.markers[i].color.b = 0.5;
// 			// _cat_marker.markers[i].color.a = 1.0; 
//     }	
//     catenary_marker_pub_.publish(_cat_marker);
//   }
}

void graphOptimizedTrajectory::goalPointMarker()
{
	visualization_msgs::Marker marker_;
	marker_.header.frame_id = "world";
	marker_.header.stamp = ros::Time();
	marker_.ns = "goal_point";
	marker_.id = 0;
	marker_.type = visualization_msgs::Marker::SPHERE;
	marker_.action = visualization_msgs::Marker::ADD;
	marker_.lifetime = ros::Duration(0);
	marker_.pose.position.x = offset_map_dll_x;
	marker_.pose.position.y = offset_map_dll_y;
	marker_.pose.position.z = offset_map_dll_z;
	marker_.pose.orientation.x = 0.0;
	marker_.pose.orientation.y = 0.0;
	marker_.pose.orientation.z = 0.0;
	marker_.pose.orientation.w = 1.0;
	marker_.scale.x = 0.4;
	marker_.scale.y = 0.4;
	marker_.scale.z = 0.4;
	marker_.color.r = 1.0;
	marker_.color.g = 0.0;
	marker_.color.b = 0.0;
	marker_.color.a = 1.0; 
	
	goal_point_pub_.publish(marker_);
}
	
int main(int argc, char **argv){
	if (argc < 2) {
		std::cout << "Usage: " << argv[0] << " <out_file> " << std::endl;
		return -1;
	}
    ros::init(argc, argv,"Trajectory_Graph_Mission_Node");
    ros::NodeHandlePtr nh;
    ros::NodeHandle pnh("~");

    graphOptimizedTrajectory got(nh, pnh);
		std::cout << "Loading FIle: " << got.file_yaml << std::endl;
    got.readWaypoints(got.file_yaml);

    while (ros::ok()) {
      ros::spinOnce();
      got.markerPoints();
      got.goalPointMarker();
	    ros::Duration(5.0).sleep();
    }	

	return 0;
}