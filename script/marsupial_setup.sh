read -p "Marsupial setup. Press a ENTER to contine. (CTRL+C) to cancel"

#! /bin/bash

sudo apt-get update

sudo apt-get install libgoogle-glog-dev

sudo apt-get install ros-$ROS_DISTRO-octomap ros-$ROS_DISTRO-octomap-ros ros-$ROS_DISTRO-octomap-msgs
sudo apt-get install ros-$ROS_DISTRO-mav-msgs ros-$ROS_DISTRO-mavlink ros-$ROS_DISTRO-mavros ros-$ROS_DISTRO-mav-comm 
