read -p "Installing optimizer for marsupial robot configuration based in g2o. Press a ENTER to contine. (CTRL+C) to cancel"

#! /bin/bash

# Dependencies for Behavior Tree
sudo apt-get install qtbase5-dev libqt5svg5-dev 
sudo apt install libdw-dev

cd ~/marsupial_ws/src

# git clone https://github.com/robotics-upo/marsupial_g2o.git

#To install planner lazy-theta*
git clone https://github.com/robotics-upo/lazy_theta_star_planners.git

# To install mcl3d
git clone https://github.com/robotics-upo/mcl3d.git
git clone https://github.com/robotics-upo/range_msgs.git

#To get action for actionlib
git clone https://github.com/robotics-upo/upo_actions.git

#To get a marker in the desired frame_link
git clone https://github.com/robotics-upo/upo_markers.git

#To get .bt form gazebo world
sim_gazebo_plugins # Tengo que subir este paquete

#To install Behavior tree
sudo apt-get install libqt5svg5-dev ros-$ROS_DISTRO-ros-type-introspection

git clone https://github.com/robotics-upo/Groot.git
git clone -b develop https://github.com/robotics-upo/behavior_tree_ros.git
git clone -b mbzirc https://github.com/robotics-upo/behavior_tree_plugins.git
git clone https://github.com/robotics-upo/BehaviorTree.CPP.git

