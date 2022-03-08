#!/bin/sh

read -p "Installation PACKAGES for platforms in MARSUPIAL SYSTEM. Press a ENTER to contine. (CTRL+C) to cancel"

sudo apt-get install ros-$ROS_DISTRO-nmea-msgs

cd ~/marsupial_ws/src/

# For Sensors

# Installing OUSTER package
echo "\n Installing UPO ouster \n\n"
git clone https://github.com/robotics-upo/ouster_upo.git

# For localization stuff
echo "\n Installing DLL \n\n"
git clone -noetic https://github.com/robotics-upo/dll.git
echo "\n Installing odom_to_tf \n\n"
git clone https://github.com/robotics-upo/odom_to_tf.git

# For Tracking
echo "\n Installing matrice_traj_tracker \n\n"
git clone https://github.com/robotics-upo/matrice_traj_tracker.git
echo "\n Installing Onboard-SDK-ROS \n\n"
git clone -b 3.8 https://github.com/dji-sdk/Onboard-SDK-ROS.git

# To get action for actionlib
echo "\n Installing UPO Actions \n\n"
git clone https://github.com/robotics-upo/upo_actions.git