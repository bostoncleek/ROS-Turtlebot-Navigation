Boston Cleek

# Overview
This package contains an implementation of model predictive path integral control (MPPI) for a TurtleBot3. The average control update occurs at a rate of 50Hz.

The controls are the left and right wheel velocities. The cost function used is that same as the LQR cost. Currently, `fake_diff_encoders` and `odometer` nodes are used to simulate the robot.

# MPPI Algorithm
An iterative path integral control update law is applied followed by a
generalized importance sampling term. Optimization and execution occur simultaneously. A trajectory is optimized and then a single control is executed. The trajectory is re-optimized using the un-executed portion of the previous trajectory.

Control perturbations are sampled from a zero mean normal distribution with specified sampling variance. The trajectories for a kinematic differential drive robot are propagated forward using a 4th order Runge-Kutta method. The cost function used is the same as the LQR cost. There is a matrix Q that penalizes the error in states, a matrix R penalizes controls, and a matrix P1 penalizes the terminal error. The cost at each state in the trajectory is a summation of the loss at each future state. Meaning early control decisions are more costly than future decisions.




# How to run
The `nuturtle_robot` package is required for the waypoint following demo. It contains the `mppi_waypoints` node. You can set the waypoints in `real_waypoints.yaml` in the `nuturtle_robot` package.

`roslaunch nuturtle_robot mppi_waypoints.launch`

The call the start service
`rosservice call /start "direction: ''"`


# Results
The following gif is at x5 speed.
<p align="center">
  <img src="media/mppi_waypoints.gif" width="500" height="300"/>
</p>


See a real time video of [MPPI waypoints](https://youtu.be/i_kzNi5Exsc).



# Functionality
The user specifies waypoints which are used as a reference. Each waypoint has a position
and a heading.

# Files
* mppi_params.yaml: cost function and control sampling parameters
* rk4.hpp/rk4.cpp: 4th order Runge-Kutta integration method  
* mppi.hpp/mppi.cpp: MPPI control algorithm

# Resources
* Williams, Grady, Andrew Aldrich, and Evangelos Theodorou. "Model predictive path integral control using covariance variable importance sampling." arXiv preprint arXiv:1509.01149 (2015).

* Abraham, Ian, et al. "Model-Based Generalization Under Parameter Uncertainty Using Path Integral Control." IEEE Robotics and Automation Letters 5.2 (2020): 2864-2871.
