
## Boston Cleek

# Description
The tsim packages uses feedforward control or bang-bang control to move the turtle in turtlesim gui
in a rectangular trajectory. The `turtle_rect` node publishes controls on `cmd_vel` and the errors
in the pose are published on `pose_error`. The service `traj_reset` restarts the
turtle back at the lower left corner.

# Commands
To launch turtlesim and the `turtle_rect` node using rqt_plot:

```
roslaunch tsim trect.launch plot_gui:=True
```

To restar the turtle at the beginning:

```
rosservice call /traj_reset
```

# Files
* tsim_node.cpp - the implementation of the control loop and the `turtle_rect` node
* turtle_params.yaml - parameters for the trajectory
* trect.launch - launches the simulation
* PoseError.msg - custom message containing the pose error
* CMakeLists.txt - cmake for tsim package
* package.xml - package xml


# Results

Here is a [video of the turtle](https://youtu.be/xHwiSVRySiA) completing one cycle around the trajectory and near the end I call the `/traj_reset` service.

Error plot of one full trajectory using the feedforward controller.

<p align="center">
  <img src= "images/err.png" width="600" height="200">
</p>

The turtle completing one trajectory using feedforward control.

<p align="center">
  <img src= "images/ff.gif" width="300" height="300">
</p>

The turtle completing one trajectory using bang-bang control.

<p align="center">
  <img src= "images/bb.gif" width="300" height="300">
</p>
