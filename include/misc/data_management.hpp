#ifndef DATA_MANAGEMENT_HPP
#define DATA_MANAGEMENT_HPP

#include <iostream>
#include <vector>

#include <ros/ros.h>

class DataManagement
{
	public:
		DataManagement(){};
		~DataManagement(){};
		void clearMarkersPointLines(visualization_msgs::MarkerArray &p_m_, visualization_msgs::MarkerArray &l_m_, ros::Publisher traj_m_pub_, auto _s);

	protected:

	private:

};

void DataManagement::clearMarkersPointLines(visualization_msgs::MarkerArray &p_m_, visualization_msgs::MarkerArray &l_m_, ros::Publisher traj_m_pub_, auto _s)
{
	l_m_.markers.clear();
	p_m_.markers.clear();

	l_m_.markers.resize(_s);
	p_m_.markers.resize(_s);

	for (auto i = 0 ; i < _s; i++){
		p_m_.markers[i].action = visualization_msgs::Marker::DELETEALL;
		l_m_.markers[i].action = visualization_msgs::Marker::DELETEALL;
	}
	traj_m_pub_.publish(p_m_);
	traj_m_pub_.publish(l_m_);
}

#endif