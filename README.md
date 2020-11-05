# marsupial_g2o

This package provides a framework to solve non-linear optimization problem for 3D trajectory planning in a marsupial robot configuration. Specifically, the configuration consists of an unmanned aerial vehicle (UAV) tied to an unmanned ground vehicle (UGV). The result is a collision-free trajectory for UAV and tether.

The optimization assumes static UGV position and estimates the problem states such as UAV and tether related with: UAV  position,  tether length, and UAV trajectory time. 

Different components such as UAV and tether positions, distance obstacles and temporal aspects of the motion (velocities and accelerations) are encoded as constraint and objective function. In consequence, the problem determining the values of the states optimizing a weighted multi-objective function.

The components encoded as constraint and objective function are local with respect to the problem states. Thus, the optimization is solved by formulating the problem as a sparse factor graph. For that g2o is used as the engine for graph optimization [https://github.com/RainerKuemmerle/g2o].

## Documentation

## Installation

In this section you will find the installation instructions for making it work. Next section (prerequisites) tells you the environment in which the package has been tested.

### Prerrequisites and dependencies

This package has been designed and tested in a x86_64 machine under a Linux 18.04 operating system and ROS Melodic distribution. Besides, our update depends script lets you easilly install the following dependencies:

- G2O
- PCL
- lazy_theta_star
- upo_actions
- behavior_tree

### Instalation steps:

1- Clone this repository into the source of your catkin workspace. Please refer to http://wiki.ros.org/catkin/Tutorials/create_a_workspace to setup a new workspace.

2- Call the g2o_installation.sh script to install G2O required dependencies.

```
rosrun marsupial_g2o g2o_installation.sh
```

3- Finally call marsupial_installation.sh script to install marsupial_g2o package.

```
rosrun marsupial_g2o marsupial_installation.sh
```

## Usage


## Optional extras
