#include "marsupial_g2o/optimizer_local_planner.h"


using namespace std;

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "marsupial_node");

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  ROS_INFO("Starting MARSUPIAL_NODE for Optimization trajectory");
  

  OptimizerLocalPlanner OLPlanner_(&tfBuffer);
  // ros::Rate r(ros::Duration(10));
  ros::Rate loop_rate(5);

  dynamic_reconfigure::Server<marsupial_g2o::OptimizationParamsConfig> server_;
  dynamic_reconfigure::Server<marsupial_g2o::OptimizationParamsConfig>::CallbackType f_;

  f_ = boost::bind(&OptimizerLocalPlanner::dynRecCb,&OLPlanner_,  _1, _2);  
  server_.setCallback(f_);

  while (ros::ok()) 
  {
    ros::spinOnce();  
    // m_.executeOptimization();
    loop_rate.sleep();
  }
  
  return 0;
  
}

