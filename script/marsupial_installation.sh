#!/bin/sh

read -p "Installing optimizer for marsupial robot configuration based in g2o. Press a ENTER to contine. (CTRL+C) to cancel"

#! /bin/bash

sudo apt-get install ros-$ROS_DISTRO-octomap-msgs ros-$ROS_DISTRO-octomap-ros
sudo apt-get install ros-$ROS_DISTRO-costmap-2d
sudo apt-get intall libpcl-dev libpcl libpcl-kdtree1.10

# For arduino (https://maker.pro/arduino/tutorial/how-to-use-arduino-with-robot-operating-system-ros)
# sudo apt-get install ros-$ROS_DISTRO-rosserial
# sudo apt-get install ros-$ROS_DISTRO-rosserial-arduino
# http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup
# git clone -b melodic-devel https://github.com/ros-drivers/rosserial.git


cd ~/marsupial_ws/src

# git clone https://github.com/robotics-upo/marsupial_g2o.git



# To install mcl3d
# https://github.com/robotics-upo/mcl3d.git
# https://github.com/robotics-upo/range_msgs.git



#To get .bt form gazebo world
# sim_gazebo_plugins # Tengo que subir este paquete

#To install Behavior tree
## Dependencies for Behavior Tree
echo "\n Installing BT Dependencies \n\n"
sudo apt-get install ros-$ROS_DISTRO-ros-type-introspection
sudo apt-get install qtbase5-dev libqt5svg5-dev 
sudo apt install libdw-dev

echo "\n Installing BT Packages \n\n"
git clone https://github.com/robotics-upo/behavior-tree-upo-actions.git
git clone -b mbzirc https://github.com/robotics-upo/BehaviorTree.CPP.git
git clone -b v2 https://github.com/robotics-upo/Groot.git

#To install planner lazy-theta*
echo "\n Installing Lazy Theta* Planner \n\n"
git clone https://github.com/robotics-upo/lazy_theta_star_planners.git

#To install Random planners
echo "\n Installing Random Planner \n\n"
git clone https://github.com/SaimonMR/rrt_star_planners.git

#To get action for actionlib
echo "\n Installing UPO Actions \n\n"
git clone https://github.com/robotics-upo/upo_actions.git

#To get a marker in the desired frame_link
echo "\n Installing UPO Markers \n\n"
git clone https://github.com/robotics-upo/upo_markers.git

# CERES Solver Installation
echo "\n Installing CERES \n\n"
## CMake
sudo apt-get install cmake
## google-glog + gflags
sudo apt-get install libgoogle-glog-dev libgflags-dev
## BLAS & LAPACK
sudo apt-get install libatlas-base-dev
## Eigen3
sudo apt-get install libeigen3-dev
## SuiteSparse and CXSparse (optional)
sudo apt-get install libsuitesparse-dev -y
cd ~
git clone https://ceres-solver.googlesource.com/ceres-solver
# tar zxf ceres-solver-2.0.0.tar.gz
mkdir ceres-bin
cd ceres-bin
# cmake ../ceres-solver-2.0.0
cmake ../ceres-solver
make -j3
make test
## Optionally install Ceres, it can also be exported using CMake which
## allows Ceres to be used without requiring installation, see the documentation
## for the EXPORT_BUILD_DIR option for more information.
sudo make install