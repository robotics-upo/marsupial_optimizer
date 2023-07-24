# marsupial_optimizer

This package provides a framework to solve non-linear optimization problem for 3D trajectory planning in a marsupial robotic system consisting of an unmanned aerial vehicle (UAV) linked to an unmanned ground vehicle (UGV) through a non-taut tether with controllable length. The objective is to determine a synchronized collision-free trajectory for the three marsupial system agents: UAV, UGV, and tether (https://youtu.be/N-K3yT8Tsxw).

<p align="center">
    <img src="worlds/repo_marsupial-optimizer.gif" width="400">
</p>
<p align="center">
    <img src="worlds/repo_marsupial-optimizer2.gif" width="400">
</p>

To the best of our knowledge, this is the first method that addresses  the trajectory planning of a marsupial UGV-UAV with a non-taut tether. The optimizer input is  a trajectory calculated adding temporal aspect to a path, wihich is computing using a planner based on optimal Rapidly-exploring Random Trees (RRT*) algorithm with novel sampling and steering techniques to speed up the computation. This algorithm is able to obtain collision-free paths for the UAV and the UGV, taking into account the 3D environment and the tether.

The trajectory optimization is based on non-linear least squares. The optimizer takes into account  aspects not considered in the path planning, like temporal constraints of the motion imposed by limits on the velocities and accelerations of the robots, trajectory smoothness, obstacles-free, or raising the tether's clearance. 

The optimization process is based on framework ceres-solver (http://ceres-solver.org/)

## Installation

In this section you will find the installation instructions for making it work. The next section (prerequisites) tells you the environment in which the package has been tested.

### Prerrequisites and dependencies

This package has been designed and tested in a x86_64 machine under a Ubuntu 20.04 operating system and ROS Noetic distribution. Besides, the scripts provided lets you easily install the following dependencies:

- ceres-solver
- PCL
- yaml-cpp
- rrt-planner ( , branch: )
- catenary_checker (https://github.com/robotics-upo/catenary_checker , branch: master) 
- upo_actions (https://github.com/robotics-upo/upo_actions, branch: master)
- upo_markers (https://github.com/robotics-upo/upo_markers, branch: master)

### Installation steps:

1- Clone this repository into the source of your catkin workspace. Please refer to http://wiki.ros.org/catkin/Tutorials/create_a_workspace to setup a new workspace.

2- Call marsupial_setup.sh script from ```marsupial_g2o/script``` directory to install package dependencies.

```
rosrun marsupial_optimizer marsupial_setup.sh
```

3- Call the ceres_installation.sh script to install Ceres-Solver required dependencies (will be install in ```/home/$user/```).

```
rosrun marsupial_optimizer ceres_installation.sh
```

4- Finally compile your workspace using ```catkin_make``` 

## Usage

Five scenarios with different features can be set to use the optimizer. S1: Open environment, S2: Narrow/constrained environment, S3: Confined environment, S4: Confined environment, S5: Open environment, as show in the next figure.

<p align="center">
    <img src="worlds/5_scenarios.png" width="1000">
</p>

The package has a set of predefined configurations (and completely extendable according to the user's need) that relate the stage number, initial position number and goal position number. The set of initial positions can be check in ```/cfg``` and the goal positions in ```/trees/resources/```.

To launch the optimizer just launch the provided ```launch/marsupial_optimization_trayectory.launch``` file. To manage the scenario and initial position predefined is recommend to use the parameter for this launch, ```scenario_number``` and ```num_pos_initial```. Thus, for example to use S2 and initial position 2: 

```
roslaunch marsupial_g2o marsupial_optimization_trayectory.launch scenario_number:=2 num_pos_initial:=2
```

It will launch the optimizer and the visualization of the environment and marsupial robots in RVIZ. 

To start the optimization process is necessary to publish a desired goal position in the topic ```/Make_Plan/goal```.
