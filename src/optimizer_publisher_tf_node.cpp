#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <string>
#include <boost/algorithm/string.hpp>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>

#include "marsupial_optimizer/marsupial_trajectory_optimized.h"
#include "misc/marker_publisher.h"
#include "misc/catenary_solver_ceres.hpp"
#include "misc/bisection_catenary_3D.h"

#define PRINTF_BLUE "\x1B[34m"

class ManagerTf{

public:
	ManagerTf(ros::NodeHandlePtr nh, ros::NodeHandle pnh);
	void initializationTf();
	void trajectoryOptimizedCallBack(const marsupial_optimizer::marsupial_trajectory_optimizedConstPtr &msg);
	void tfBroadcaster(const double _x,const double _y,const double _z,const double _R,const double _P,const double _Y, std::string _from, std::string _to);
	void getReelPose();
	geometry_msgs::Vector3 getReelPoint(const float px_, const float py_, const float pz_,const float qx_, const float qy_, const float qz_, const float qw_);
	
	marsupial_optimizer::marsupial_trajectory_optimized msg;
    marsupial_optimizer::marsupial_trajectory_optimized trajectory; 
	tf::TransformBroadcaster br;
	tf::Transform trans_ugv, trans_uav;
    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> tf2_list;
    std::unique_ptr<tf::TransformListener> tf_list_ptr;
	
	// ros::NodeHandlePtr nh;
    // ros::NodeHandle pnh("~");
	ros::Subscriber trajectory_sub_;
	ros::Publisher finished_rviz_maneuver_pub_, catenary_marker_pub_;
	std::string uav_base_frame, ugv_base_frame, reel_base_frame;
	
	geometry_msgs::TransformStamped pose_reel_local;

	int num_pos_initial;
	double initial_pos_ugv_x, initial_pos_ugv_y, initial_pos_ugv_z, initial_pos_ugv_yaw, initial_pos_ugv_pitch, initial_pos_ugv_roll;
	double initial_pos_uav_x, initial_pos_uav_y, initial_pos_uav_z, initial_pos_uav_yaw, initial_pos_uav_pitch, initial_pos_uav_roll;
	double pos_uav_above_ugv;
	bool publish_initial;

	MarkerPublisher mp_;
	visualization_msgs::MarkerArray catenary_marker;

protected:

};

ManagerTf::ManagerTf(ros::NodeHandlePtr nh, ros::NodeHandle pnh)
{
	printf("\n\tInitialazing Publisher_TF_NODE !!\n");
	
	nh.reset(new ros::NodeHandle("~"));
  	
	tfBuffer.reset(new tf2_ros::Buffer);
    tf2_list.reset(new tf2_ros::TransformListener(*tfBuffer));
    tf_list_ptr.reset(new tf::TransformListener(ros::Duration(5)));

	pnh.param<int>("num_pos_initial", num_pos_initial, 1);
	
	std::string n_, n_test;
	n_ = std::to_string(num_pos_initial);
	n_test = "test_"+n_+"/";

	pnh.param<double>(n_test+"initial_pos_ugv_x", initial_pos_ugv_x, 0.0);
	pnh.param<double>(n_test+"initial_pos_ugv_y", initial_pos_ugv_y, 0.0);
	pnh.param<double>(n_test+"initial_pos_ugv_z", initial_pos_ugv_z, 0.0);
	pnh.param<double>(n_test+"initial_pos_ugv_roll", initial_pos_ugv_roll, 0.0);
	pnh.param<double>(n_test+"initial_pos_ugv_pitch", initial_pos_ugv_pitch, 0.0);
	pnh.param<double>(n_test+"initial_pos_ugv_yaw", initial_pos_ugv_yaw, 0.0001);
	pnh.param<double>(n_test+"initial_pos_uav_x", initial_pos_uav_x, 0.0);
	pnh.param<double>(n_test+"initial_pos_uav_y", initial_pos_uav_y, 0.0);
	pnh.param<double>(n_test+"initial_pos_uav_z", initial_pos_uav_z, 0.0);
	pnh.param<double>(n_test+"initial_pos_uav_roll", initial_pos_uav_roll, 0.0);
	pnh.param<double>(n_test+"initial_pos_uav_pitch", initial_pos_uav_pitch, 0.0);
	pnh.param<double>(n_test+"initial_pos_uav_yaw", initial_pos_uav_yaw, 0.0001);
	pnh.param<double>("pos_uav_above_ugv",pos_uav_above_ugv, 1.0);  

	pnh.param("uav_base_frame", uav_base_frame, (std::string)"uav_base_link");
  	pnh.param("ugv_base_frame", ugv_base_frame, (std::string)"ugv_base_link");
  	pnh.param("reel_base_frame", reel_base_frame, (std::string)"reel_base_link");

    trajectory_sub_ = nh->subscribe("/trajectory_optimized", 2000, &ManagerTf::trajectoryOptimizedCallBack,this);
    finished_rviz_maneuver_pub_ = nh->advertise<std_msgs::Bool>("/finished_rviz_maneuver", 1);
	catenary_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("catenary_marker", 100);

	publish_initial = true;

	printf("\tINITIAL POS UGV: trans=[%f %f %f] rot=[%f %f %f]\n",initial_pos_ugv_x, initial_pos_ugv_y, initial_pos_ugv_z, initial_pos_ugv_roll, initial_pos_ugv_pitch, initial_pos_ugv_yaw);
	printf("\t**INITIAL POS UAV: trans=[%f %f %f] rot=[%f %f %f]\n",initial_pos_uav_x, initial_pos_uav_y, initial_pos_uav_z, initial_pos_uav_roll, initial_pos_uav_pitch, initial_pos_uav_yaw);
}

void ManagerTf::initializationTf()
{
	if(publish_initial){
		tfBroadcaster(initial_pos_ugv_x, initial_pos_ugv_y, initial_pos_ugv_z, initial_pos_ugv_roll, initial_pos_ugv_pitch, initial_pos_ugv_yaw, "/map", ugv_base_frame);
		tfBroadcaster(initial_pos_ugv_x, initial_pos_ugv_y, initial_pos_ugv_z+pos_uav_above_ugv, initial_pos_ugv_roll, initial_pos_ugv_pitch, initial_pos_ugv_yaw, "/map", uav_base_frame);
	}
	else{
		br.sendTransform(tf::StampedTransform(trans_ugv, ros::Time::now(), "/map", ugv_base_frame));
		br.sendTransform(tf::StampedTransform(trans_uav, ros::Time::now(), "/map", uav_base_frame));
	}
	mp_.clearMarkers(catenary_marker, 150, catenary_marker_pub_);
	getReelPose();
}

void ManagerTf::trajectoryOptimizedCallBack(const marsupial_optimizer::marsupial_trajectory_optimizedConstPtr &msg)
{
	trajectory.trajectory = msg->trajectory;
	trajectory.length_catenary = msg->length_catenary;
    ROS_INFO(PRINTF_BLUE "RECEIVED TRAJECTORY: Preparing to execute RViz maneuver");
	publish_initial = false;

	tf::Vector3 v_ugv, v_uav;
	tf::Quaternion q_ugv, q_uav;

	// CatenarySolver cSX_;
	bisectionCatenary bc;

	// cSX_.setMaxNumIterations(100);
	std::vector<geometry_msgs::Point> points_catenary_;

	for (size_t i= 0; i < trajectory.trajectory.points.size() ; i++){
		double ugv_x = trajectory.trajectory.points.at(i).transforms[0].translation.x;
		double ugv_y = trajectory.trajectory.points.at(i).transforms[0].translation.y;
		double ugv_z = trajectory.trajectory.points.at(i).transforms[0].translation.z;
		double ugv_rot_x = trajectory.trajectory.points.at(i).transforms[0].rotation.x;
		double ugv_rot_y = trajectory.trajectory.points.at(i).transforms[0].rotation.y;
		double ugv_rot_z = trajectory.trajectory.points.at(i).transforms[0].rotation.z;
		double ugv_rot_w = trajectory.trajectory.points.at(i).transforms[0].rotation.w;
		double uav_x = trajectory.trajectory.points.at(i).transforms[1].translation.x;
		double uav_y = trajectory.trajectory.points.at(i).transforms[1].translation.y;
		double uav_z = trajectory.trajectory.points.at(i).transforms[1].translation.z;
		double uav_rot_x = trajectory.trajectory.points.at(i).transforms[1].rotation.x;
		double uav_rot_y = trajectory.trajectory.points.at(i).transforms[1].rotation.y;
		double uav_rot_z = trajectory.trajectory.points.at(i).transforms[1].rotation.z;
		double uav_rot_w = trajectory.trajectory.points.at(i).transforms[1].rotation.w;
		printf("UGV=[%.3f %.3f %.3f %.3f] UAV=[%.3f %.3f %.3f %.3f]\n", ugv_rot_x ,ugv_rot_y ,ugv_rot_z ,ugv_rot_w ,uav_rot_x ,uav_rot_y ,uav_rot_z ,uav_rot_w);
		v_ugv = tf::Vector3(ugv_x, ugv_y, ugv_z);
		q_ugv = tf::Quaternion(ugv_rot_x ,ugv_rot_y ,ugv_rot_z ,ugv_rot_w);
		trans_ugv.setOrigin(v_ugv);
		trans_ugv.setRotation(q_ugv);
		v_uav = tf::Vector3(uav_x, uav_y, uav_z);
		q_uav = tf::Quaternion(uav_rot_x ,uav_rot_y ,uav_rot_z ,uav_rot_w);
		trans_uav.setOrigin(v_uav);
		trans_uav.setRotation(q_uav);

		//Graph final catenary states
		points_catenary_.clear();
		geometry_msgs::Vector3 p_reel_ =  getReelPoint(ugv_x, ugv_y, ugv_z, ugv_rot_x ,ugv_rot_y ,ugv_rot_z ,ugv_rot_w);
		
		//Solve through Ceres
			// cSX_.solve(p_reel_.x, p_reel_.y, p_reel_.z, uav_x, uav_y, uav_z, trajectory.length_catenary[i], points_catenary_);
		//Solver through Bisection
			bool just_one_axe = bc.configBisection(trajectory.length_catenary[i], p_reel_.x, p_reel_.y, p_reel_.z, uav_x, uav_y, uav_z, false);
			bc.getPointCatenary3D(points_catenary_);
		
		auto size_ = points_catenary_.size();
		mp_.clearMarkers(catenary_marker, 150, catenary_marker_pub_);
		
		mp_.markerPoints(catenary_marker, points_catenary_, i, size_, catenary_marker_pub_);	
		br.sendTransform(tf::StampedTransform(trans_ugv, ros::Time::now(), "/map", ugv_base_frame));
		br.sendTransform(tf::StampedTransform(trans_uav, ros::Time::now(), "/map", uav_base_frame));

		ros::Duration(1.0).sleep();
		printf("Rviz Trajectory: [%lu/%lu]: UGV=[%.3f %.3f %.3f / %.3f %.3f %.3f %.3f] UAV=[%.3f %.3f %.3f / %.3f %.3f %.3f %.3f] length=[%.3f]\n", i,trajectory.trajectory.points.size(),
		ugv_x ,ugv_y ,ugv_z ,ugv_rot_x ,ugv_rot_y ,ugv_rot_z ,ugv_rot_w ,uav_x ,uav_y ,uav_z ,uav_rot_x ,uav_rot_y ,uav_rot_z ,uav_rot_w,trajectory.length_catenary[i]);
	}
	ROS_INFO(PRINTF_BLUE "FINISHED TRAJECTORY: RViz maneuver");
	std_msgs::Bool finished_rviz_maneuver;
	finished_rviz_maneuver.data = true;
	finished_rviz_maneuver_pub_.publish(finished_rviz_maneuver); 
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

void ManagerTf::getReelPose()
{
	try{
        pose_reel_local = tfBuffer->lookupTransform(ugv_base_frame, reel_base_frame, ros::Time(0));
    }catch (tf2::TransformException &ex){
        // ROS_WARN("Optimizer Local Planner: Couldn't get Local Reel Pose (frame: %s), so not possible to set Tether start point; tf exception: %s", reel_base_frame.c_str(),ex.what());
    }
	// printf("ManagerTf::getReelPose()  , pose_reel_local=[%f %f %f] \n",pose_reel_local.transform.translation.x, pose_reel_local.transform.translation.y,pose_reel_local.transform.translation.z);
}

geometry_msgs::Vector3 ManagerTf::getReelPoint(const float px_, const float py_, const float pz_,const float qx_, const float qy_, const float qz_, const float qw_)
{
	geometry_msgs::Vector3 ret;

	double roll_, pitch_, yaw_;
	tf::Quaternion q_(qx_,qy_,qz_,qw_);
	tf::Matrix3x3 M_(q_);	
	M_.getRPY(roll_, pitch_, yaw_);

	double lengt_vec =  sqrt(pose_reel_local.transform.translation.x*pose_reel_local.transform.translation.x + pose_reel_local.transform.translation.y*pose_reel_local.transform.translation.y);
	ret.x = px_ + lengt_vec *cos(yaw_); 
	ret.y = py_ + lengt_vec *sin(yaw_);
	ret.z = pz_ + pose_reel_local.transform.translation.z ;

	return ret;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "optmizer_publisher_tf_node");

	ros::NodeHandlePtr nh;
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
