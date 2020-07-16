#include "marsupial.hpp"
#include "ros/ros.h"


// ============= Global Variables ================ 


// =========== Function declarations =============



// =============== Main function =================

using namespace std;

int main(int argc, char** argv) {
  ros::init(argc, argv, "marsupial_node");
  
  ROS_INFO("Starting MARSUPIAL Optimization trajectory");
  
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  Marsupial m_(nh, pnh);
  ros::Rate r(ros::Duration(10));
  
  while (ros::ok()) 
  {
    ros::spinOnce();  
  }
  
  return 0;
  
}