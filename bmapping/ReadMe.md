Boston Cleek

# Description
The goal of this project was to implement rao-blackwellized particle filter SLAM from scratch. This package was developed and tested on Ubuntu 18.04 running ROS Melodic.


# Dependencies
* Eigen
* Point Cloud Library

# How to Download
1. `wstool init`
2. `wstool merge nuturtle.rosinstall`
3. `wstool update`
4. `catkin_make`

# How to Run
Use the turtlebot teleop keys to move around and run:
`roslaunch bmapping slam.launch`


# Nodes



# Files



# Results


# Algorithms
* The Rao-Blackwellized Particle Filter Algorithm from ["Improved Techniques for Grid Mapping with Rao-Blackwellized Particle Filters"](http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti07tro.pdf)
* Iterative Closest Point - Point Cloud Library ICP solver
* Occupancy Grid Mapping - Probabilistic Robotics Table 9.1
* Updated the Occupancy Grid using Raycasting [Bresenham's line algorithm](https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm)
* Pose Likelihood - Probabilistic Robotics Table 5.5
* Scan Likelihood - Probabilistic Robotics Table 6.3
