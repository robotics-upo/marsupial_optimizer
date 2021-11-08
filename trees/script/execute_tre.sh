#!/bin/bash

arg1="$1"
echo $arg1
path1="\"tree_file:"
path2="'"
path3="/home/simon/marsupial_ws/src/marsupial_optimizer/trees/"

echo  $path1 $path2$path3
name_file=$path2$path3$arg1
echo $path1 $name_file.xml"'\""

rosservice call /behavior_tree/load_tree $path1 $name_file


