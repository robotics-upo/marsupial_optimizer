#include "marsupial_g2o/optimizer_publisher_tf.hpp"

ManagerTf::ManagerTf() 
{
	ROS_INFO("Initialazing optimizer_publisher_tf_node");
	_nh->param<double>("initial_pos_ugv_x", initial_pos_ugv_x, 0.0);
	_nh->param<double>("initial_pos_ugv_y", initial_pos_ugv_y, 0.0);
	_nh->param<double>("initial_pos_ugv_z", initial_pos_ugv_z, 0.0);
	_nh->param<double>("initial_pos_ugv_yaw", initial_pos_ugv_yaw, 0.0);
	_nh->param<double>("initial_pos_ugv_pitch", initial_pos_ugv_pitch, 0.0);
	_nh->param<double>("initial_pos_ugv_roll", initial_pos_ugv_roll, 0.0);

	_nh->param<double>("pos_uav_above_ugv",pos_uav_above_ugv, 1.0);  

	_nh->param("uav_base_frame", uav_base_frame, (std::string)"uav_base_link");
  	_nh->param("ugv_base_frame", ugv_base_frame, (std::string)"ugv_base_link");
	
	printf("init.trans=[%f %f %f] init.rot=[%f %f %f]\n",initial_pos_ugv_x,initial_pos_ugv_y,initial_pos_ugv_z,initial_pos_ugv_roll,initial_pos_ugv_pitch,initial_pos_ugv_yaw);
	
	initializationTf();
}

void ManagerTf::initializationTf()
{
	tfBroadcaster(initial_pos_ugv_x, initial_pos_ugv_y, initial_pos_ugv_z, initial_pos_ugv_roll, initial_pos_ugv_pitch, initial_pos_ugv_yaw, "/map", ugv_base_frame);
	tfBroadcaster(initial_pos_ugv_x, initial_pos_ugv_y, initial_pos_ugv_z+pos_uav_above_ugv, initial_pos_ugv_roll, initial_pos_ugv_pitch, initial_pos_ugv_yaw, "/map", uav_base_frame);
}

void ManagerTf::tfBroadcaster(const double _x,const double _y,const double _z,const double _R,const double _P,const double _Y, std::string _from, std::string _to)
{
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(_x, _y, _z) );
	tf::Quaternion q;
	q.setRPY(_R, _P, _Y);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), _from, _to));
}