read -p "Installing optimizer for marsupial robot configuration based in g2o. Press a ENTER to contine. (CTRL+C) to cancel"

#! /bin/bash

cd ~/marsupial_ws/src

# git clone https://github.com/robotics-upo/marsupial_g2o.git

#To install planner lazy-theta*
git clone https://github.com/robotics-upo/lazy_theta_star_planners.git

# To install mcl3d
https://github.com/robotics-upo/mcl3d.git
https://github.com/robotics-upo/range_msgs.git

#To get action for actionlib
https://github.com/robotics-upo/upo_actions.git

#To get a marker in the desired frame_link
https://github.com/robotics-upo/upo_markers.git

#To get .bt form gazebo world
sim_gazebo_plugins