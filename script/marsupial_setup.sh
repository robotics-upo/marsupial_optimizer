read -p "Installing package dependencies marsupial optimizer. Press a ENTER to contine. (CTRL+C) to cancel"

#! /bin/bash


cd ../..

#To install planner lazy-theta 3D*
git clone https://github.com/robotics-upo/lazy_theta_star_planners.git

#To get action for actionlib
git clone https://github.com/robotics-upo/upo_actions.git

#To get a marker in the desired frame_link
git clone https://github.com/robotics-upo/upo_markers.git





