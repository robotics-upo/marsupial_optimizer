#include "marsupial_optimizer/optimizer_local_planner.h"


using namespace std;

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "marsupial_node");
  google::InitGoogleLogging(argv[0]);

  
  ROS_INFO("Starting MARSUPIAL_NODE for Optimization trajectory");

  OptimizerLocalPlanner OLPlanner_;
  
  ros::Rate loop_rate(5);

  dynamic_reconfigure::Server<marsupial_optimizer::OptimizationParamsConfig> server_;
  dynamic_reconfigure::Server<marsupial_optimizer::OptimizationParamsConfig>::CallbackType f_;

  f_ = boost::bind(&OptimizerLocalPlanner::dynRecCb,&OLPlanner_,  _1, _2);  
  server_.setCallback(f_);

  while (ros::ok()) 
  {
    ros::spinOnce();  
    loop_rate.sleep();
  }
  
  return 0;
}

