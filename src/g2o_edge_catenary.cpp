#include "marsupial_g2o/g2o_edge_catenary.h"

namespace g2o {

	G2OCatenaryEdge::G2OCatenaryEdge(ros::NodeHandlePtr nhP) :
		BaseBinaryEdge<3, geometry_msgs::Point, VertexPointXYZ, VertexCatenaryFactor>() 
	{
		_information.setIdentity();
		_error.setZero();
	    catenary_marker_pub_ = nhP->advertise<visualization_msgs::MarkerArray>("catenary_marker", 2);

	}

	bool G2OCatenaryEdge::read(std::istream& is)
	{
		geometry_msgs::Point p;
		is >> p.x;
		is >> p.y;
		is >> p.z;
		setMeasurement(p);
		for (int i = 0; i < 3; ++i)
			for (int j = i; j < 3; ++j) {
				is >> information()(i, j);
				if (i != j)
					information()(j, i) = information()(i, j);
		}
		return true;
	}

	bool G2OCatenaryEdge::write(std::ostream& os) const
	{
		geometry_msgs::Point p = measurement();
		os << p.x;
		os << p.y;
		os << p.z;
		for (int i = 0; i < 3; ++i)
			for (int j = i; j < 3; ++j)
				os << " " << information()(i, j);
		return os.good();
	}

	inline void G2OCatenaryEdge::setLengthCatenary(double _v){_length_catenary = _v;} 

	void G2OCatenaryEdge::markerPoints(visualization_msgs::MarkerArray _marker, std::vector<geometry_msgs::Point> _vector, int _suffix, int _n_v)
	{
		std::string string_marker;
		std::string ns_marker;

		double c_color1 = (_suffix / (double)_n_v)* 0.5;
		double c_color2 = (_suffix / (double)_n_v)* 0.5;
		// printf ("c_color = [%f] _suffix=[%i]  _n_v=[%i]\n",c_color, _suffix, _n_v);
	  	
		string_marker = std::to_string(_suffix);
		ns_marker = "catenary_"+string_marker;

		_marker.markers.resize(_vector.size());
		
		for (size_t i = 0; i < _vector.size(); ++i){
			_marker.markers[i].header.frame_id = "/map";
			_marker.markers[i].header.stamp = ros::Time::now();
			_marker.markers[i].ns = ns_marker;
			_marker.markers[i].id = i+1;
			_marker.markers[i].action = visualization_msgs::Marker::ADD;
			_marker.markers[i].type = visualization_msgs::Marker::SPHERE;
			_marker.markers[i].lifetime = ros::Duration(400);
			// printf("markerPoints : _marker[%lu].pose.position [%f %f %f]\n",i,_vector[i].x,_vector[i].y,_vector[i].z);
			_marker.markers[i].pose.position.x = _vector[i].x; 
			_marker.markers[i].pose.position.y = _vector[i].y; 
			_marker.markers[i].pose.position.z = _vector[i].z;

			_marker.markers[i].pose.orientation.x = 0.0;
			_marker.markers[i].pose.orientation.y = 0.0;
			_marker.markers[i].pose.orientation.z = 0.0;
			_marker.markers[i].pose.orientation.w = 1.0;
			_marker.markers[i].scale.x = 0.08;
			_marker.markers[i].scale.y = 0.08;
			_marker.markers[i].scale.z = 0.08;
			_marker.markers[i].color.a = 1.0;
			_marker.markers[i].color.r = 0.9;
			_marker.markers[i].color.g = c_color1;
			_marker.markers[i].color.b = 0.0;
		}	
		catenary_marker_pub_.publish(_marker);
	}

	void G2OCatenaryEdge::clearMarkers(visualization_msgs::MarkerArray _marker,auto _s)
	{
		_marker.markers.clear();
		_marker.markers.resize(_s);

		for (auto i = 0 ; i < _s; i++){
			_marker.markers[i].action = visualization_msgs::Marker::DELETEALL;
		}
		catenary_marker_pub_.publish(_marker);
	}

} // end namespace