#!/bin/bash

arg1="$HOME"
arg2="$1"

echo "Set parameter: <tree_name.xml>"
echo "    pc_user_name: $arg1"
echo "    tree_name: $arg2"

path="$HOME/marsupial_ws/src/marsupial_optimizer/trees/"

full_path="$path$arg2"
echo "full_path: $full_path"
#rosservice call /behavior_tree/load_tree "tree_file: '$full_path'"
rostopic pub /behavior_tree/load_tree_action/goal behavior_tree_ros/BehaviorTreeActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  tree_file:
    data: '$full_path'" --once

