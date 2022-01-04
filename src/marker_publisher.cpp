#include "misc/marker_publisher.h"

void MarkerPublisher::markerPoints(visualization_msgs::MarkerArray _marker, std::vector<geometry_msgs::Point> _vector, int _suffix, int _n_v, ros::Publisher c_m_pub_, int change_marker_)
{
    std::string string_marker;
    std::string ns_marker;

    double c_color1 = (_suffix / (double)_n_v)*0.5;
    double c_color2 = (_suffix / (double)_n_v)*0.5;
    double c_color3;
    if (_suffix%2 == 0)
        c_color3 = 0.5;
    else
        c_color3 = 0.0;
	
	double _scale = 0.05;

	if(change_marker_==1){
		c_color1 = 0.0;
		c_color2 = 1.0;
		c_color3 = 1.0;
		_scale = 0.045;
	}
	else if(change_marker_==2){
		c_color1 = 0.0;
		c_color2 = 0.0;
		c_color3 = 0.0;
		_scale = 0.045;
	}
            
    string_marker = std::to_string(_suffix);
    ns_marker = "catenary_"+string_marker;

    _marker.markers.resize(_vector.size());
            
    for (size_t i = 0; i < _vector.size(); ++i){
        _marker.markers[i].header.frame_id = "map";
        _marker.markers[i].header.stamp = ros::Time::now();
        _marker.markers[i].ns = ns_marker;
        _marker.markers[i].id = i+1;
        _marker.markers[i].action = visualization_msgs::Marker::ADD;
        if (i % 5 == 0 || change_marker_!=0)
            _marker.markers[i].type = visualization_msgs::Marker::CUBE;
        else
            _marker.markers[i].type = visualization_msgs::Marker::SPHERE;
        _marker.markers[i].lifetime = ros::Duration(0);
        _marker.markers[i].pose.position.x = _vector[i].x; 
        _marker.markers[i].pose.position.y = _vector[i].y; 
        _marker.markers[i].pose.position.z = _vector[i].z;

        _marker.markers[i].pose.orientation.x = 0.0;
        _marker.markers[i].pose.orientation.y = 0.0;
        _marker.markers[i].pose.orientation.z = 0.0;
        _marker.markers[i].pose.orientation.w = 1.0;
        _marker.markers[i].scale.x = _scale;
        _marker.markers[i].scale.y = _scale;
        _marker.markers[i].scale.z = _scale;
        _marker.markers[i].color.a = 1.0;
        _marker.markers[i].color.r = 1.0 - c_color1;
        _marker.markers[i].color.g = c_color2;
        _marker.markers[i].color.b = c_color3;
    }	
    c_m_pub_.publish(_marker);
}

void MarkerPublisher::clearMarkers(visualization_msgs::MarkerArray _marker, int _s, ros::Publisher c_m_pub_)
{
    _marker.markers.clear();
    _marker.markers.resize(_s);

    for (int i = 0 ; i < _s; i++){
        _marker.markers[i].action = visualization_msgs::Marker::DELETEALL;
    }
    c_m_pub_.publish(_marker);
}

void MarkerPublisher::getMarkerPoints(visualization_msgs::MarkerArray &marker_, std::vector<geometry_msgs::Vector3> vector_, std::string ns_, int colour_)
{
	// RED: colour_ = 0 ; GREEN : colour_ = 1 ; BLUE: colour_ = 2 ; YELLOW = 3 ; PURPLE = 4; ; BLACK = 5 ; WHITE = 6
	for (size_t i = 0; i < vector_.size(); i++){
		marker_.markers[i].header.frame_id = "map";
		marker_.markers[i].header.stamp = ros::Time::now();
		marker_.markers[i].ns = ns_;
		marker_.markers[i].id = i+1;
		marker_.markers[i].action = visualization_msgs::Marker::ADD;
		if (i % 5 == 0)
			marker_.markers[i].type = visualization_msgs::Marker::CUBE;
		else
			marker_.markers[i].type = visualization_msgs::Marker::SPHERE;
		marker_.markers[i].lifetime = ros::Duration(0);
		marker_.markers[i].pose.position.x = vector_[i].x;
		marker_.markers[i].pose.position.y = vector_[i].y;
		marker_.markers[i].pose.position.z = vector_[i].z;
		marker_.markers[i].pose.orientation.x = 0.0;
		marker_.markers[i].pose.orientation.y = 0.0;
		marker_.markers[i].pose.orientation.z = 0.0;
		marker_.markers[i].pose.orientation.w = 1.0;
		marker_.markers[i].scale.x = 0.2;
		marker_.markers[i].scale.y = 0.2;
		marker_.markers[i].scale.z = 0.2;
		marker_.markers[i].color.a = 1.0;
		if (colour_ == 0){
			marker_.markers[i].color.r = 0.9;
			marker_.markers[i].color.g = 0.0;
			marker_.markers[i].color.b = 0.0;
		}
		if (colour_ == 1){
			marker_.markers[i].color.r = 0.0;
			marker_.markers[i].color.g = 0.9;
			marker_.markers[i].color.b = 0.0;
		}
		if (colour_ == 2){
			marker_.markers[i].color.r = 0.0;
			marker_.markers[i].color.g = 0.0;
			marker_.markers[i].color.b = 0.9;
		}
		if (colour_ == 3){
			marker_.markers[i].color.r = 0.9;
			marker_.markers[i].color.g = 0.9;
			marker_.markers[i].color.b = 0.0;
		}
		if (colour_ == 4){
			marker_.markers[i].color.r = 0.9;
			marker_.markers[i].color.g = 0.0;
			marker_.markers[i].color.b = 0.9;
		}
		if (colour_ == 5){
			marker_.markers[i].color.r = 0.0;
			marker_.markers[i].color.g = 0.0;
			marker_.markers[i].color.b = 0.0;
		}
		if (colour_ == 6){
			marker_.markers[i].color.r = 1.0;
			marker_.markers[i].color.g = 1.0;
			marker_.markers[i].color.b = 1.0;
		}
	}	
}

void MarkerPublisher::getMarkerLines(visualization_msgs::MarkerArray &marker_, std::vector<geometry_msgs::Vector3> _vector, std::string ns_, int colour_)
{
	// RED: colour_ = 0 ; GREEN : colour_ = 1 ; BLUE: colour_ = 2 ; YELLOW = 3 ; PURPLE = 4;
	for (size_t i = 0; i < _vector.size()-1; i++){
		geometry_msgs::Point _p1, _p2; 
		marker_.markers[i].header.frame_id = "map";
		marker_.markers[i].header.stamp = ros::Time::now();
		marker_.markers[i].ns = ns_;
		marker_.markers[i].id = i + _vector.size()+2;
		marker_.markers[i].action = visualization_msgs::Marker::ADD;
		marker_.markers[i].type = visualization_msgs::Marker::LINE_STRIP;
		marker_.markers[i].lifetime = ros::Duration(0);
		_p1.x = _vector[i].x;
		_p1.y = _vector[i].y;
		_p1.z = _vector[i].z;
		marker_.markers[i].points.push_back(_p1);
		_p2.x = _vector[i+1].x;
		_p2.y = _vector[i+1].y;
		_p2.z = _vector[i+1].z;
		marker_.markers[i].points.push_back(_p2);
		marker_.markers[i].pose.orientation.x = 0.0;
		marker_.markers[i].pose.orientation.y = 0.0;
		marker_.markers[i].pose.orientation.z = 0.0;
		marker_.markers[i].pose.orientation.w = 1.0;
		marker_.markers[i].scale.x = 0.1;
		// marker_.markers[i].scale.y = 0.3;
		// marker_.markers[i].scale.z = 0.1;
		marker_.markers[i].color.a = 1.0;
		if (colour_ == 0){
			marker_.markers[i].color.r = 0.9;
			marker_.markers[i].color.g = 0.0;
			marker_.markers[i].color.b = 0.9;
		}
		if (colour_ == 1){
			marker_.markers[i].color.r = 0.0;
			marker_.markers[i].color.g = 0.9;
			marker_.markers[i].color.b = 0.0;
		}
		if (colour_ == 2){
			marker_.markers[i].color.r = 0.0;
			marker_.markers[i].color.g = 0.0;
			marker_.markers[i].color.b = 0.9;
		}
		if (colour_ == 3){
			marker_.markers[i].color.r = 0.9;
			marker_.markers[i].color.g = 0.9;
			marker_.markers[i].color.b = 0.0;
		}
		if (colour_ == 4){
			marker_.markers[i].color.r = 0.9;
			marker_.markers[i].color.g = 0.0;
			marker_.markers[i].color.b = 0.9;
		}
	}
}

void MarkerPublisher::clearMarkersPointLines(visualization_msgs::MarkerArray &p_m_, visualization_msgs::MarkerArray &l_m_, ros::Publisher traj_m_pub_, int _s)
{
	l_m_.markers.clear();
	p_m_.markers.clear();

	l_m_.markers.resize(_s);
	p_m_.markers.resize(_s);

	for (int i = 0 ; i < _s; i++){
		p_m_.markers[i].action = visualization_msgs::Marker::DELETEALL;
		l_m_.markers[i].action = visualization_msgs::Marker::DELETEALL;
	}
	traj_m_pub_.publish(p_m_);
	traj_m_pub_.publish(l_m_);
}

void MarkerPublisher::initialAndFinalPointsMarker(geometry_msgs::Vector3 v3_i, geometry_msgs::Vector3 v3_f, ros::Publisher initial_final_pub_)
{
	visualization_msgs::MarkerArray marker_nearest;
	marker_nearest.markers.resize(2);
	marker_nearest.markers[0].header.frame_id = "map";
	marker_nearest.markers[0].header.stamp = ros::Time();
	marker_nearest.markers[0].ns = "initial_point";
	marker_nearest.markers[0].id = 0;
	marker_nearest.markers[0].type = visualization_msgs::Marker::SPHERE;
	marker_nearest.markers[0].action = visualization_msgs::Marker::ADD;
	marker_nearest.markers[0].lifetime = ros::Duration(0);
	marker_nearest.markers[0].pose.position.x = v3_i.x;
	marker_nearest.markers[0].pose.position.y = v3_i.y;
	marker_nearest.markers[0].pose.position.z = v3_i.z;
	marker_nearest.markers[0].pose.orientation.x = 0.0;
	marker_nearest.markers[0].pose.orientation.y = 0.0;
	marker_nearest.markers[0].pose.orientation.z = 0.0;
	marker_nearest.markers[0].pose.orientation.w = 1.0;
	marker_nearest.markers[0].scale.x = 0.1;
	marker_nearest.markers[0].scale.y = 0.1;
	marker_nearest.markers[0].scale.z = 0.1;
	marker_nearest.markers[0].color.r = 0.1;
	marker_nearest.markers[0].color.g = 1.0;
	marker_nearest.markers[0].color.b = 0.1;
	marker_nearest.markers[0].color.a = 1.0; 

	marker_nearest.markers[1].header.frame_id = "map";
	marker_nearest.markers[1].header.stamp = ros::Time();
	marker_nearest.markers[1].ns = "final_point";
	marker_nearest.markers[1].id = 1;
	marker_nearest.markers[1].type = visualization_msgs::Marker::SPHERE;
	marker_nearest.markers[1].action = visualization_msgs::Marker::ADD;
	marker_nearest.markers[1].lifetime = ros::Duration(0);
	marker_nearest.markers[1].pose.position.x = v3_f.x;
	marker_nearest.markers[1].pose.position.y = v3_f.y;
	marker_nearest.markers[1].pose.position.z = v3_f.z;
	marker_nearest.markers[1].pose.orientation.x = 0.0;
	marker_nearest.markers[1].pose.orientation.y = 0.0;
	marker_nearest.markers[1].pose.orientation.z = 0.0;
	marker_nearest.markers[1].pose.orientation.w = 1.0;
	marker_nearest.markers[1].scale.x = 0.1;
	marker_nearest.markers[1].scale.y = 0.1;
	marker_nearest.markers[1].scale.z = 0.1;
	marker_nearest.markers[1].color.r = 0.1;
	marker_nearest.markers[1].color.g = 0.1;
	marker_nearest.markers[1].color.b = 1.0;
	marker_nearest.markers[1].color.a = 1.0; 
	
	initial_final_pub_.publish(marker_nearest);
}