#include <ros/ros.h>
#include "misc/grid3d.hpp"
#include <string>


Grid3d *grid_3D;


void createGrid(std::string node_name_){

grid_3D = new Grid3d(node_name_);

}

int main(int argc, char **argv)
{
    std::string node_name = "create_grid_node";

	ros::init(argc, argv, node_name);


    ros::spinOnce();
    createGrid(node_name);


    return 0;
}