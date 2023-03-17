#!/bin/bash

arg1="$USER"
arg2="$2"

echo "Set parameter: <tree_name.xml>"
echo "    pc_user_name: $arg1"
echo "    tree_name: $arg2"

path="/home/$arg1/marsupial_ws/src/marsupial_optimizer/trees/"

full_path="$path$arg2"
echo "full_path: $full_path"
rosservice call /behavior_tree/load_tree "tree_file: '$full_path'"
