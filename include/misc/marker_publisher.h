#ifndef MARKER_PUBLISHER_H_
#define MARKER_PUBLISHER_H_

#include <iostream>
#include <fstream>
#include <vector>

// #include <g2o/core/factory.h>
#include <iomanip>
#include <Eigen/StdVector>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>

class MarkerPublisher
{
	public:
		MarkerPublisher(){};
		~MarkerPublisher(){};
		void markerPoints(visualization_msgs::MarkerArray _marker, std::vector<geometry_msgs::Vector3> _vector, int _suffix, int _n_v, ros::Publisher c_m_pub_, int change_marker_= 0);
		void clearMarkers(visualization_msgs::MarkerArray _marker, int _s, ros::Publisher c_m_pub_);
		void getMarkerPoints(visualization_msgs::MarkerArray &marker_, std::vector<geometry_msgs::Vector3> _vector, std::string ns, int colour_);
		void getMarkerLines(visualization_msgs::MarkerArray &marker_, std::vector<geometry_msgs::Vector3> _vector, std::string ns, int colour_);
		void clearMarkersPointLines(visualization_msgs::MarkerArray &p_m_, visualization_msgs::MarkerArray &l_m_, ros::Publisher traj_m_pub_, int _s);
		void initialAndFinalPointsMarker(geometry_msgs::Vector3 v3_i, geometry_msgs::Vector3 v3_f, ros::Publisher nearest_point_pub_);

	protected:

	private:
};

#endif