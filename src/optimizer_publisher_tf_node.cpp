// #include "marsupial_optimizer/optimizer_publisher_tf.hpp"

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <string>
#include <boost/algorithm/string.hpp>

class ManagerTf{

public:
	ManagerTf(ros::NodeHandle _nh, ros::NodeHandle _pnh);
	std::string uav_base_frame, ugv_base_frame;
	int num_pos_initial;
	double initial_pos_ugv_x, initial_pos_ugv_y, initial_pos_ugv_z, initial_pos_ugv_yaw, initial_pos_ugv_pitch, initial_pos_ugv_roll, pos_uav_above_ugv;
	void initializationTf();
	void tfBroadcaster(const double _x,const double _y,const double _z,const double _R,const double _P,const double _Y, std::string _from, std::string _to);

protected:

};


ManagerTf::ManagerTf(ros::NodeHandle _nh, ros::NodeHandle _pnh)
{
	printf("Initialazing optimizer_publisher_tf_node !!\n");
  	_pnh.param<int>("num_pos_initial", num_pos_initial, 1);
	
	std::string n_, n_test;
	n_ = std::to_string(num_pos_initial);
	n_test = "test_"+n_+"/";

	_nh.param<double>(n_test+"initial_pos_ugv_x", initial_pos_ugv_x, 0.0);
	_nh.param<double>(n_test+"initial_pos_ugv_y", initial_pos_ugv_y, 0.0);
	_nh.param<double>(n_test+"initial_pos_ugv_z", initial_pos_ugv_z, 0.0);
	_nh.param<double>(n_test+"initial_pos_ugv_yaw", initial_pos_ugv_yaw, 0.0);
	_nh.param<double>(n_test+"initial_pos_ugv_pitch", initial_pos_ugv_pitch, 0.0);
	_nh.param<double>(n_test+"initial_pos_ugv_roll", initial_pos_ugv_roll, 0.0);

	_pnh.param<double>("pos_uav_above_ugv",pos_uav_above_ugv, 1.0);  

	_pnh.param("uav_base_frame", uav_base_frame, (std::string)"uav_base_link");
  	_pnh.param("ugv_base_frame", ugv_base_frame, (std::string)"ugv_base_link");

	
	printf("init.trans=[%f %f %f] init.rot=[%f %f %f]\n",initial_pos_ugv_x,initial_pos_ugv_y,initial_pos_ugv_z,initial_pos_ugv_roll,initial_pos_ugv_pitch,initial_pos_ugv_yaw);
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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "optmizer_publisher_tf_node");

	ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ManagerTf mtf(nh,pnh);

	// ros::Rate r(ros::Duration(0.05));
    
	while (ros::ok()) {
        ros::spinOnce();
		mtf.initializationTf();
        // r.sleep();
    }	
	
	return 0;

}
