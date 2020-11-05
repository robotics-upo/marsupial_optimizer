# About marsupial_g2o

This package provides a framework to solve non-linear optimization problem for 3D trajectory planning in a marsupial robot configuration. Specifically, the configuration consists of an unmanned aerial vehicle (UAV) tied to an unmanned ground vehicle (UGV). The result is a collision-free trajectory for UAV and tether.

The optimization assumes static UGV position and estimates the problem states such as UAV and tether related with: UAV  position,  tether length, and UAV trajectory time. 

Different components such as UAV and tether positions, distance obstacles and temporal aspects of the motion (velocities and accelerations) are encoded as constraint and objective function. In consequence, the problem determining the values of the states optimizing a weighted multi-objective function.

The components encoded as constraint and objective function are local with respect to the problem states. Thus, the optimization is solved by formulating the problem as a sparse factor graph. For that g2o is used as the engine for graph optimization [https://github.com/RainerKuemmerle/g2o].

# Documentation

# Installation

This package contains the implementation of a "g2o-based optimizer" to obtain the optimized navigation path of a drone tied with a cable to a ground robot, a robotic configuration known as marsupial. The optimizer is based on C ++, ROS Melodic and g2o.
