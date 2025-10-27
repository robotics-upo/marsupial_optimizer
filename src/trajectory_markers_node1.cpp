#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <vector>
#include <iostream>

// Tu clase de publicación de markers (tal como la compartiste)
#include "misc/manage_computed_path.hpp"
#include "misc/marker_publisher.h"   // Ruta/nombre según tu árbol de include
#include "catenary_checker/catenary_checker_manager.h"

// ^ Métodos usados: getMarkerPoints / getMarkerLines  (ver impl. en marker_publisher.cpp)

// (Opcional) Solo si usas la alternativa A2 para leer YAML directamente aquí
#include <yaml-cpp/yaml.h>
#include "misc/vector_to_point.hpp"

geometry_msgs::Point getReelPoint(const float px_, const float py_, const float pz_,const float qx_, const float qy_, const float qz_, const float qw_, geometry_msgs::TransformStamped pose_reel_local_);
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
int main(int argc, char** argv)
{
  std::string node_name = "trajectory_markers_node";
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~"); // private ns para leer parámetros ~param
  std::shared_ptr<tf2_ros::Buffer> tfBuffer;
  std::unique_ptr<tf2_ros::TransformListener> tf2_list;

  tfBuffer.reset(new tf2_ros::Buffer);
  tf2_list.reset(new tf2_ros::TransformListener(*tfBuffer));

  // Parámetros
  std::string path;                 // p.ej. "/home/simon-msi/traj/"
  std::string name_file_trajectory; // p.ej. "datos"
  double distance_obstacle_ugv, distance_obstacle_uav, distance_tether_obstacle, length_tether_max, ws_z_min, map_resolution;
  bool use_distance_function, use_tether, use_catenary_as_tether, just_line_of_sight;
  std::string ugv_base_frame, uav_base_frame, reel_base_frame, map_path;

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
  std::string file_name = path +"/"+ name_file_trajectory + ".yaml";
  std::cout << "Oppening Trajectory from : " << file_name << std::endl;

	Grid3d *grid_3D; //, *grid_3D_obst, *grid_3D_trav;
  grid_3D = new Grid3d(node_name, map_path);
  	ROS_INFO_COND(true, PRINTF_BLUE "Initialazing Trilinear Interpolation (grid3D) in Optimizer");
	grid_3D->computeTrilinearInterpolation();
  	ROS_INFO_COND(true, PRINTF_BLUE "Finished Trilinear Interpolation (grid3D) in Optimizer");

  // Build ManagePath
  ManagePath MP(file_name);

  // Create Vectors
  std::vector<geometry_msgs::Point> vector_ugv_;
  std::vector<geometry_msgs::Point> vector_uav_;
  std::vector<double> vector_length_;
	CatenaryCheckerManager *CheckCM;

  geometry_msgs::TransformStamped pose_reel_local;
  try{
      pose_reel_local = tfBuffer->lookupTransform(ugv_base_frame, reel_base_frame, ros::Time(0));
  }catch (tf2::TransformException &ex){
      ROS_WARN("Optimizer Local Planner: Couldn't get Local Reel Pose (frame: %s), so not possible to set Tether start point; tf exception: %s", reel_base_frame.c_str(),ex.what());
  }

  CheckCM = new CatenaryCheckerManager(node_name);
  CheckCM->init(grid_3D, distance_tether_obstacle, distance_obstacle_ugv, distance_obstacle_uav, length_tether_max, ws_z_min, step,
                use_tether, use_distance_function, toPoint(pose_reel_local.transform.translation), just_line_of_sight, use_catenary_as_tether);
  ros::spinOnce();
  ROS_INFO("Waiting 5.0 seconds...");
  ros::Duration(5.0).sleep();  // pausa exacta en segundos
  ROS_INFO("Continue...");

  //     Loading path values in vectors
  double total_dist_ , avg_dist_uav_ , avg_dist_ugv_ , min_dist_uav_ , min_dist_ugv_;
  total_dist_ = avg_dist_uav_ = avg_dist_ugv_ = 0.0;
  min_dist_uav_ = min_dist_ugv_ = 1000.0;
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
        total_dist_ = total_dist_ + d_;
        if (d_ < min_dist_ugv_)
          min_dist_ugv_ = d_;
      }
    }
    avg_dist_ugv_ = total_dist_/vector_ugv_.size();
    
    // UAV
    const YAML::Node& uav = root["marsupial_uav"];
    total_dist_ = 0.0;
    for (auto it = uav.begin(); it != uav.end(); ++it) {
      const std::string key = it->first.as<std::string>();
      if (key.rfind("poses", 0) == 0) {
        geometry_msgs::Point p;
        const auto& pose = it->second["pose"]["position"];
        p.x = pose["x"].as<double>();
        p.y = pose["y"].as<double>();
        p.z = pose["z"].as<double>();
        double d_ = CheckCM->getPointDistanceObstaclesMap(true, p);
        vector_uav_.push_back(p);
        total_dist_ = total_dist_ + d_;
        if (d_ < min_dist_uav_)
          min_dist_uav_ = d_;
      }
    }
    avg_dist_uav_ = total_dist_/vector_uav_.size();
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
    return 1;
  }

  printf("size vector_ugv_ = %lu\n",vector_ugv_.size());
  for(int i = 0 ; i < vector_ugv_.size(); i++){
    printf("\t [%i] UAV=(%f %f %f) UGV=(%f %f %f) L=(%f)\n",
    i, vector_uav_[i].x,vector_uav_[i].y,vector_uav_[i].z, vector_ugv_[i].x,vector_ugv_[i].y,vector_ugv_[i].z, vector_length_[i]);
  }

  // Publishers de markers (uno por *set* de arrays; puedes unificar si lo prefieres)
  ros::Publisher ugv_points_markers_pub = nh.advertise<visualization_msgs::MarkerArray>("ugv_points_markers", 1, true);
  ros::Publisher ugv_lines_markers_pub = nh.advertise<visualization_msgs::MarkerArray>("ugv_lines_markers", 1, true);
  ros::Publisher uav_points_markers_pub = nh.advertise<visualization_msgs::MarkerArray>("uav_points_markers", 1, true);
  ros::Publisher uav_lines_markers_pub = nh.advertise<visualization_msgs::MarkerArray>("uav_lines_markers", 1, true);
  ros::Publisher tether_markers_pub = nh.advertise<visualization_msgs::MarkerArray>("tether_markers", 1, true);

  // Objetos MarkerArray a publicar
  visualization_msgs::MarkerArray points_ugv_marker, lines_ugv_marker;
  visualization_msgs::MarkerArray points_uav_marker, lines_uav_marker;
  visualization_msgs::MarkerArray catenary_marker;

  // Instancia del publicador de markers
  MarkerPublisher MPub;

  // Colores (según tu implem.: 0=RED, 1=GREEN, 2=BLUE, 3=YELLOW, 4=PURPLE, 5=BLACK, 6=WHITE)
  int c_ugv_ = 1; // azul para UGV, por ejemplo
  int c_uav_ = 2; // rojo  para UAV

  // Construcción de markers con los vectores
  // (Usamos la API exacta de tu MarkerPublisher)  :contentReference[oaicite:2]{index=2} :contentReference[oaicite:3]{index=3}
  MPub.getMarkerPoints(points_ugv_marker, vector_ugv_, "points_ugv_m", c_ugv_);
  MPub.getMarkerLines (lines_ugv_marker,  vector_ugv_, "lines_ugv_m",  c_ugv_);

  MPub.getMarkerPoints(points_uav_marker, vector_uav_, "points_uav_m", c_uav_);
  MPub.getMarkerLines (lines_uav_marker,  vector_uav_, "lines_uav_m",  c_uav_);

	GetTetherParameter GTP_;
	vector<geometry_msgs::Point> points_catenary_; 
  for(int i = 0 ; i < vector_length_.size(); i++){
    points_catenary_.clear();
		geometry_msgs::Point p_reel_ = getReelPoint(vector_ugv_[i].x,vector_ugv_[i].y,vector_ugv_[i].z, 0, 0, 0, 1, pose_reel_local);
	  
    CheckCM->searchCatenary(p_reel_, vector_uav_[i], points_catenary_);
	
    MPub.markerPoints(catenary_marker, points_catenary_, i, points_catenary_.size(), tether_markers_pub, 1, false);	
  }

  // Publica una vez (latched publishers) o dentro de un loop si deseas refrescar el stamp
  ugv_points_markers_pub.publish(points_ugv_marker);
  ugv_lines_markers_pub.publish(lines_ugv_marker);

  uav_points_markers_pub.publish(points_uav_marker);
  uav_lines_markers_pub.publish(lines_uav_marker);

  ROS_INFO_STREAM("Publicados " << points_ugv_marker.markers.size() << " puntos UGV y "
                                << lines_ugv_marker.markers.size()  << " segmentos UGV.");
  ROS_INFO_STREAM("Publicados " << points_uav_marker.markers.size() << " puntos UAV y "
                                << lines_uav_marker.markers.size()  << " segmentos UAV.");
  
  // Mantén vivo el nodo si quieres visualizar en RViz con stamps actuales
  ros::Rate r(1.0);
  while (ros::ok()) {
    // Si prefieres refrescar marcas con stamp actual:
    // MPub.getMarkerPoints(points_ugv_marker, vector_ugv_, "points_ugv_m", c_ugv_);
    // MPub.getMarkerLines (lines_ugv_marker,  vector_ugv_, "lines_ugv_m",  c_ugv_);
    // ugv_markers_pub.publish(points_ugv_marker);
    // ugv_markers_pub.publish(lines_ugv_marker);
    //
    // MPub.getMarkerPoints(points_uav_marker, vector_uav_, "points_uav_m", c_uav_);
    // MPub.getMarkerLines (lines_uav_marker,  vector_uav_, "lines_uav_m",  c_uav_);
    // uav_markers_pub.publish(points_uav_marker);
    // uav_markers_pub.publish(lines_uav_marker);

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
