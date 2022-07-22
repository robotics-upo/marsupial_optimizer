#ifndef _PUBLISHER_TF_HPP_
#define _PUBLISHER_TF_HPP_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <string>
#include <boost/algorithm/string.hpp>
#include <geometry_msgs/Point.h>

class ManagerTf
{
	public:
		ManagerTf() ;
		ros::NodeHandlePtr _nh;
		std::string uav_base_frame, ugv_base_frame;
		double initial_pos_ugv_x, initial_pos_ugv_y, initial_pos_ugv_z, initial_pos_ugv_yaw, initial_pos_ugv_pitch, initial_pos_ugv_roll, pos_uav_above_ugv;
		void initializationTf();
		void tfBroadcaster(const double _x,const double _y,const double _z,const double _R,const double _P,const double _Y, std::string _from, std::string _to);
		void markerPoints(visualization_msgs::MarkerArray _marker, std::vector<geometry_msgs::Point> _vector, int _suffix, int _n_v, int change_marker_);

	protected:

};

#endif