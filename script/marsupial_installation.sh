read -p "Installing optimizer for marsupial robot configuration based in g2o. Press a ENTER to contine. (CTRL+C) to cancel"

#! /bin/bash



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
git clone -b mbzirc https://github.com/robotics-upo/behavior_tree_plugins.git
git clone -b mbzirc https://github.com/robotics-upo/behavior_tree_ros.git
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
