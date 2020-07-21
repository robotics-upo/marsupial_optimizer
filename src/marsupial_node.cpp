#include "marsupial.hpp"
#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>
#include "marsupial_g2o/OptimizationParamsConfig.h"

// ============= Global Variables ================ 


// =========== Function declarations =============

void dynamicReconfigureCallback(marsupial_g2o::OptimizationParamsConfig &config, uint32_t level){
  ROS_INFO("Dynamic reconfigure callback");
}

// =============== Main function =================

using namespace std;

int main(int argc, char** argv) {
  ros::init(argc, argv, "marsupial_node");
  
  ROS_INFO("Starting MARSUPIAL Optimization trajectory");
  
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  Marsupial m_(nh, pnh, "marsupial_node");
  // ros::Rate r(ros::Duration(10));
  ros::Rate r(5);

  dynamic_reconfigure::Server<marsupial_g2o::OptimizationParamsConfig> server_;
  dynamic_reconfigure::Server<marsupial_g2o::OptimizationParamsConfig>::CallbackType f_;

  f_ = boost::bind(&dynamicReconfigureCallback,_1,_2);
  server_.setCallback(f_);

  while (ros::ok()) 
  {
    ros::spinOnce();  
    // m_.executeOptimization();
    r.sleep();
  }
  
  return 0;
  
}

