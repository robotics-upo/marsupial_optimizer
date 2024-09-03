#pragma once

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

inline geometry_msgs::Point toPoint(const geometry_msgs::Vector3 &v) {
    geometry_msgs::Point ret;
    ret.x = v.x; ret.y = v.y; ret.z = v.z;

    return ret;
}

inline geometry_msgs::Vector3 toVector3(const geometry_msgs::Point &p) {
    geometry_msgs::Vector3 ret;
    ret.x = p.x; ret.y = p.y; ret.z = p.z;

    return ret;
}