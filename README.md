Boston Cleek

# ROS TurtleBot Navigation from Scratch

# Description
The goal is to create a fully autonomous navigation stack for a TurtleBot from scratch.

This repository contains rao-blackwellized particle filter slam, EKF SLAM (known and unknown data association), several global path planners (Theta* , D* light, Potential Field), and control (MPPI and proportional feedback).

Currently this repo contains the necessary software to run both SLAM implementations on an actual TurtleBot or you can run the in sim using the gazebo plugin.


# Packages
* [bmapping](https://github.com/bostoncleek/ROS-Turtlebot-Navigation/tree/master/bmapping): rao-blackwellized particle filter slam (a.k.a gmapping or Fast SLAM for grid mapping)
* [nuslam](https://github.com/bostoncleek/ROS-Turtlebot-Navigation/tree/master/nuslam): EKF SLAM and feature detection (cylindrical landmarks)
* [planner](https://github.com/bostoncleek/ROS-Turtlebot-Navigation/tree/master/planner): Global path planning in continous and discrete space
* [controller](https://github.com/bostoncleek/ROS-Turtlebot-Navigation/tree/master/controller) Model predicive path integral control for the TurtleBot using a kinematic model of a differential drive robot
* [rigi2d](https://github.com/bostoncleek/ROS-Turtlebot-Navigation/tree/master/rigid2d): 2D Lie Group transformations, vectors, twists, model of differential drive robot, singleton pattern for random number generation, waypoint following using proportional control, and nodes for modeling encoders readings and odometry
* [nuturtle_description](https://github.com/bostoncleek/ROS-Turtlebot-Navigation/tree/master/nuturtle_description): Contains URDF and config files relative to the TurtleBot3 hardware
* [nuturtle_gazebo](https://github.com/bostoncleek/ROS-Turtlebot-Navigation/tree/master/nuturtle_gazebo): Contains a gazebo plugin to emulate low level controls and lidar sensing in simulation
* [nuturtle_robot](https://github.com/bostoncleek/ROS-Turtlebot-Navigation/tree/master/nuturtle_robot): Interfaces with TurtleBot3 hardware and contains launch files for waypoint following
* [tsim](https://github.com/bostoncleek/ROS-Turtlebot-Navigation/tree/master/tsim): Nodes to test capabilites of rigid2d packages using turtlesim




# Getting Started
### 1) Required Files
Use the nuturtle.rosinstall file to clone the required packages.

`mkdir -p turtlebot_nav/src` <br/>
`cd ~/turtlebot_nav/src` <br/>
`wstool init .` <br/>
`wstool merge -t . https://raw.githubusercontent.com/bostoncleek/ROS-Turtlebot-Navigation/master/nuturtle.rosinstall` <br/>
`wstool update -t .` <br/>
`cd ~/turtlebot_nav` <br/>
`catkin_make`

### 2) Start Mapping
The two main packages are [bmapping](https://github.com/bostoncleek/ROS-Turtlebot-Navigation/tree/master/bmapping) and [nuslam](https://github.com/bostoncleek/ROS-Turtlebot-Navigation/tree/master/nuslam) check out their repo's to use both SLAM implementations.




# Future Development
* Test bmapping and nuslam on TurtleBot
* Develop a full state local planner and integration into the global planner
* Use MPPI control to traverse the local motion plan.
* Manually place waypoints by clicking on locations of the map in rviz and the TurtleBot will autonomously drive to them
* TurtleBot explores autonomously (drives to low entropy areas in map or searches for objects)
