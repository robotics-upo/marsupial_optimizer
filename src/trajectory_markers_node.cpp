#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include "misc/manage_computed_path.hpp"
#include "misc/marker_publisher.h"   
#include "catenary_checker/catenary_checker_manager.h"

#include <yaml-cpp/yaml.h>
#include "misc/vector_to_point.hpp"

std::string file_name ;

// Create Vectors
std::vector<geometry_msgs::Point> vector_ugv_, vector_uav_;
std::vector<double> vector_length_;

// Class
CatenaryCheckerManager *CheckCM;
Grid3d *grid_3D;
MarkerPublisher MPub;

geometry_msgs::Point getReelPoint(const float px_, const float py_, const float pz_,const float qx_, const float qy_, const float qz_, const float qw_, geometry_msgs::TransformStamped pose_reel_local_);
void plotPathMarker();

// Parameters
std::string path;                 
std::string name_file_trajectory; 
double distance_obstacle_ugv, distance_obstacle_uav, distance_tether_obstacle, length_tether_max, ws_z_min, map_resolution;
bool use_distance_function, use_tether, use_catenary_as_tether, just_line_of_sight;
std::string ugv_base_frame, uav_base_frame, reel_base_frame, map_path;
ros::Publisher ugv_points_markers_pub, ugv_lines_markers_pub , uav_points_markers_pub, uav_lines_markers_pub , tether_markers_pub;

// MarkerArray for publish
visualization_msgs::MarkerArray points_ugv_marker, lines_ugv_marker, points_uav_marker, lines_uav_marker, catenary_marker;
geometry_msgs::TransformStamped pose_reel_local;


geometry_msgs::Point getReelPoint(const float px_, const float py_, const float pz_,const float qx_, const float qy_, const float qz_, const float qw_, geometry_msgs::TransformStamped pose_reel_local_)
{
	geometry_msgs::Point ret;

	double roll_, pitch_, yaw_;
	tf::Quaternion q_(qx_,qy_,qz_,qw_);
	tf::Matrix3x3 M_(q_);	
	M_.getRPY(roll_, pitch_, yaw_);

	double lengt_vec =  sqrt(pose_reel_local_.transform.translation.x*pose_reel_local_.transform.translation.x + pose_reel_local_.transform.translation.y*pose_reel_local_.transform.translation.y);
	ret.x = px_ + lengt_vec *cos(yaw_); 
	ret.y = py_ + lengt_vec *sin(yaw_);
	ret.z = pz_ + pose_reel_local_.transform.translation.z ;

	return ret;
}

void plotPathMarker(){
    //     Loading path values in vectors
  std::ofstream csv_ugv("/home/simon/d_ugv.csv");  
  std::ofstream csv_uav("/home/simon/d_uav.csv");  
  std::ofstream csv_tether("/home/simon/d_tether.csv");  

  try {
    YAML::Node root = YAML::LoadFile(file_name);
    // UGV
    const YAML::Node& ugv = root["marsupial_ugv"];
    for (auto it = ugv.begin(); it != ugv.end(); ++it) {
      const std::string key = it->first.as<std::string>();
      if (key.rfind("poses", 0) == 0) {
        geometry_msgs::Point p;
        const auto& pose = it->second["pose"]["position"];
        p.x = pose["x"].as<double>();
        p.y = pose["y"].as<double>();
        p.z = pose["z"].as<double>();
        vector_ugv_.push_back(p);
        double d_ = CheckCM->getPointDistanceObstaclesMap(false, p);
        csv_ugv <<  std::setprecision(6) << d_ << "\n";
      }
    }
    
    // UAV
    const YAML::Node& uav = root["marsupial_uav"];
    for (auto it = uav.begin(); it != uav.end(); ++it) {
      const std::string key = it->first.as<std::string>();
      if (key.rfind("poses", 0) == 0) {
        geometry_msgs::Point p;
        const auto& pose = it->second["pose"]["position"];
        p.x = pose["x"].as<double>();
        p.y = pose["y"].as<double>();
        p.z = pose["z"].as<double>();
        vector_uav_.push_back(p);
        double d_ = CheckCM->getPointDistanceObstaclesMap(true, p);
        csv_uav <<  std::setprecision(6) << d_ << "\n";
      }
    }
    // Tether length
    const YAML::Node& tether = root["tether"];
    for (auto it = tether.begin(); it != tether.end(); ++it) {
      const std::string key = it->first.as<std::string>();
      if (key.rfind("length", 0) == 0) {
        vector_length_.push_back(it->second["length"].as<double>());
      }
    }
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Error leyendo YAML: " << e.what());
  }

  printf("size vector_ugv_ = %lu\n",vector_ugv_.size());
  for(int i = 0 ; i < vector_ugv_.size(); i++){
    printf("\t [%i] UAV=(%f %f %f) UGV=(%f %f %f) L=(%f)\n",
    i, vector_uav_[i].x,vector_uav_[i].y,vector_uav_[i].z, vector_ugv_[i].x,vector_ugv_[i].y,vector_ugv_[i].z, vector_length_[i]);
  }

  // Colors (0=RED, 1=GREEN, 2=BLUE, 3=YELLOW, 4=PURPLE, 5=BLACK, 6=WHITE)
  int c_ugv_ = 1; // 
  int c_uav_ = 2; // 

  MPub.getMarkerPoints(points_ugv_marker, vector_ugv_, "points_ugv_m", c_ugv_);
  MPub.getMarkerLines (lines_ugv_marker,  vector_ugv_, "lines_ugv_m",  c_ugv_);
  MPub.getMarkerPoints(points_uav_marker, vector_uav_, "points_uav_m", c_uav_);
  MPub.getMarkerLines (lines_uav_marker,  vector_uav_, "lines_uav_m",  c_uav_);

	GetTetherParameter GTP_;
	vector<geometry_msgs::Point> points_catenary_; 
  MPub.clearMarkers(catenary_marker, 200, tether_markers_pub);
  printf("size vector_length_ = %lu\n",vector_length_.size());

  for(int i = 3 ; i < vector_length_.size(); i++){
    points_catenary_.clear();
		geometry_msgs::Point p_reel_ = getReelPoint(vector_ugv_[i].x,vector_ugv_[i].y,vector_ugv_[i].z+0.35, 0, 0, 0, 1, pose_reel_local);
	  
    CheckCM->NumericalSolutionCatenary(p_reel_, vector_uav_[i], points_catenary_, vector_length_[i]);
    for(int j = 0; j < points_catenary_.size(); j++){
        double d_ = CheckCM->getPointDistanceObstaclesMap(true, points_catenary_[j]);
        csv_tether <<  std::setprecision(6) << d_ << "\n";
    }
	
    MPub.markerPoints(catenary_marker, points_catenary_, i, points_catenary_.size(), tether_markers_pub, 1, false);	
  }

  // Published just one time
  ugv_points_markers_pub.publish(points_ugv_marker);
  ugv_lines_markers_pub.publish(lines_ugv_marker);
  uav_points_markers_pub.publish(points_uav_marker);
  uav_lines_markers_pub.publish(lines_uav_marker);

  ROS_INFO_STREAM("Publicados " << points_ugv_marker.markers.size() << " puntos UGV y "
                                << lines_ugv_marker.markers.size()  << " segmentos UGV.");
  ROS_INFO_STREAM("Publicados " << points_uav_marker.markers.size() << " puntos UAV y "
                                << lines_uav_marker.markers.size()  << " segmentos UAV.");

}

int main(int argc, char** argv)
{
  std::string node_name = "trajectory_markers_node";
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~"); // private ns para leer parámetros ~param
  std::shared_ptr<tf2_ros::Buffer> tfBuffer;
  std::unique_ptr<tf2_ros::TransformListener> tf2_list;

  tfBuffer.reset(new tf2_ros::Buffer);
  tf2_list.reset(new tf2_ros::TransformListener(*tfBuffer));

  nh.param<std::string>("path", path, "");
  nh.param<std::string>("name_file_trajectory", name_file_trajectory, "datos");
  nh.param<double>("distance_obstacle_ugv", distance_obstacle_ugv,0.5);
	nh.param<double>("distance_obstacle_uav", distance_obstacle_uav,1.0);
	nh.param<double>("distance_tether_obstacle", distance_tether_obstacle, 0.1);
	nh.param<double>("length_tether_max", length_tether_max,20.0);
  nh.param<double>("ws_z_min", ws_z_min, 0.00);
	nh.param<double>("map_resolution", map_resolution,0.05);
  nh.param<bool>("use_distance_function", use_distance_function, true);
 	nh.param<bool>("use_tether", use_tether, true);
  nh.param<bool>("just_line_of_sight",just_line_of_sight, false);
	nh.param<bool>("use_catenary_as_tether",use_catenary_as_tether, true);
  nh.param("ugv_base_frame", ugv_base_frame, (std::string) "ugv_base_link");
 	nh.param("uav_base_frame", uav_base_frame, (std::string) "uav_base_link");
  nh.param("reel_base_frame", reel_base_frame, (std::string) "reel_base_link");
	nh.param("map_path", map_path, (std::string) "map.bt");


  double step = map_resolution;
  file_name = path +"/"+ name_file_trajectory + ".yaml";
  std::cout << "Oppening Trajectory from : " << file_name << std::endl;

  grid_3D = new Grid3d(node_name, map_path);
  	ROS_INFO_COND(true, PRINTF_BLUE "Initialazing Trilinear Interpolation (grid3D) in Optimizer");
	grid_3D->computeTrilinearInterpolation();
  	ROS_INFO_COND(true, PRINTF_BLUE "Finished Trilinear Interpolation (grid3D) in Optimizer");

  // Build ManagePath
  ManagePath MP(file_name);

  try{
      pose_reel_local = tfBuffer->lookupTransform(ugv_base_frame, reel_base_frame, ros::Time(0));
  }catch (tf2::TransformException &ex){
      ROS_WARN("Optimizer Local Planner: Couldn't get Local Reel Pose (frame: %s), so not possible to set Tether start point; tf exception: %s", reel_base_frame.c_str(),ex.what());
  }

  CheckCM = new CatenaryCheckerManager(node_name);
  CheckCM->init(grid_3D, distance_tether_obstacle, distance_obstacle_ugv, distance_obstacle_uav, length_tether_max, ws_z_min, step,
                use_tether, use_distance_function, toPoint(pose_reel_local.transform.translation), just_line_of_sight, use_catenary_as_tether);
  
  ROS_INFO("Waiting 5.0 seconds...");
  ros::Duration(5.0).sleep();  // pausa exacta en segundos
  ROS_INFO("Continue...");

  // Publishers de markers (uno por *set* de arrays; puedes unificar si lo prefieres)
  ugv_points_markers_pub = nh.advertise<visualization_msgs::MarkerArray>("ugv_points_markers", 1, true);
  ugv_lines_markers_pub = nh.advertise<visualization_msgs::MarkerArray>("ugv_lines_markers", 1, true);
  uav_points_markers_pub = nh.advertise<visualization_msgs::MarkerArray>("uav_points_markers", 1, true);
  uav_lines_markers_pub = nh.advertise<visualization_msgs::MarkerArray>("uav_lines_markers", 1, true);
  tether_markers_pub = nh.advertise<visualization_msgs::MarkerArray>("tether_markers", 1, true);
  
  // Mantén vivo el nodo si quieres visualizar en RViz con stamps actuales
  ros::Rate r(1.0);
  ros::spinOnce();
  r.sleep();
  plotPathMarker();
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
    // plotPathMarker();
  }

  return 0;
}
