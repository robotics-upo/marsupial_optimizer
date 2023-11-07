#include "marsupial_optimizer/optimizer_local_planner.h"

using namespace std;

bool get_path_from_file;

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "marsupial_node");
  ros::NodeHandle nh("~");	

  google::InitGoogleLogging(argv[0]);

  ROS_INFO("Starting MARSUPIAL_NODE for Optimization trajectory");

  if(!nh.getParam("get_path_from_file", get_path_from_file))
    get_path_from_file = false;

  OptimizerLocalPlanner OLPlanner_(get_path_from_file);
  
  ros::Rate loop_rate(5);

  dynamic_reconfigure::Server<marsupial_optimizer::OptimizationParamsConfig> server_;
  dynamic_reconfigure::Server<marsupial_optimizer::OptimizationParamsConfig>::CallbackType f_;

  server_.setCallback(f_);

  while (ros::ok()) 
  {
    ros::spinOnce();  
    loop_rate.sleep();
  }
  
  return 0;
}

