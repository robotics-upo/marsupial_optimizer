#!/bin/bash

arg1="$1"
arg2="$2"

echo "Set parameter: <pc_user_name> <tree_name>"
echo "    pc_user_name: $arg1"
echo "    tree_name: $arg2"

path="/home/$arg1/marsupial_ws/src/marsupial_optimizer/trees/"

full_path="$path$arg2"
echo "full_path: $full_path"
rosservice call /behavior_tree/load_tree "tree_file: '$full_path'"
