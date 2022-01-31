#!/bin/bash

arg1="$1"
echo "$arg1"

path1="/home/srlhp/marsupial_ws/src/marsupial_optimizer/trees/"

full_path="$path1$arg1"
rosservice call /behavior_tree/load_tree "tree_file: '$full_path'"
