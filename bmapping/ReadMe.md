Boston Cleek

# Description
The goal of this project was to implement rao-blackwellized particle filter SLAM from scratch. This package was developed and tested on Ubuntu 18.04 running ROS Melodic. This is a SLAM package part of a larger ROS navigation, mapping, and localization framework.


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
The `slam` node performs odometry and SLAM by subscribing to `scan`, `joint_states`, and `/gazebo/model_states` topics. The occupancy grid is published on `map` and the odometry is published on `odom`. The node also publishes the trajectories of the robot for the gazebo simulation on `gazebo_path`, the odometry trajectory on `odom_path`, and the slam trajectory on `slam_path`. The error in the robot's pose relative to the gazebo model state for odometry is published on `odom_error` and the error in the slam pose is published on `slam_error`. The `slam` node also broadcasts a `tf` from the `map` frame to the `odom` frame and another `tf` from the `odom` frame to the `base_link` frame.


# Files
* turtle_mapping_node.cpp - The `slam` node
* partilce_filter.hpp - Rao-Blackwellized Particle Filter SLAM
* grid_mapper.hpp - Occupancy Grid mapping, Raycasting, Euclidean Signed Distance Field, Scan Likelihood
* cloud_alignment.hpp - Iterative closes point using the Point Cloud Library
* sensor_model.hpp - Models the lidar sensor
* LDA_01_lidar.yaml - turtlebot3 lidar properties


# Results

The ground truth path from gazebo is orange, SLAM is blue, and odometry is green.
The final error in the pose relative to gazebo is given bellow. The map was created using 40 particles.

|          |      X Error (cm)      |  Y Error (cm) |  Yaw Error  (degrees) |
|----------|:-----------------:|---------:|-----------:|
|  Odometry  |  19.5     |   -10.5    |     2.62      |
| RBPF SLAM   |     -1.04      |   3.81    |     1.98       |

<figure align="center">
  <img src="/images/slam_map.jpg" width="300" height="300"/>
  <img src="/images/slam_path.jpg" width="300" height="300"/>
</figure>

# Algorithms
* The Rao-Blackwellized Particle Filter Algorithm from ["Improved Techniques for Grid Mapping with Rao-Blackwellized Particle Filters"](http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti07tro.pdf)
* Fast Marching Method - Creates [Euclidean Signed Distance Field](https://canvas.northwestern.edu/login/saml) (required for the scan likelihood)
* Iterative Closest Point - Point Cloud Library ICP solver
* Occupancy Grid Mapping - Probabilistic Robotics Table 9.1
* Updated the Occupancy Grid using Raycasting [Bresenham's line algorithm](https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm)
* Pose Likelihood - Probabilistic Robotics Table 5.5
* Scan Likelihood - Probabilistic Robotics Table 6.3
