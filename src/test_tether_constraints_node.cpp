#include "marsupial_optimizer/test_tether_constraints.h"

using namespace std;

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "test_theter_constraints");
	ros::NodeHandle n;
  ros::NodeHandle nh("~");	

  google::InitGoogleLogging(argv[0]);

  ROS_INFO("Starting NODE for Testing Tether Constraints");

  std::string node_name_ = "test_theter_constraints_node";
  TestTetherConstraints ttc(node_name_);

  ROS_INFO("Initialized NODE for Testing Tether Constraints");

  
  ros::Rate loop_rate(5);

  while (ros::ok()) 
  {
    ros::spinOnce();  
    loop_rate.sleep();
  }
  
  return 0;
}
