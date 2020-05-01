### Boston Cleek

# Overview:
A collection of shortest path planning algorithms in continuous and discrete space


# How to run
For the continuous map theta* planner:

`roslaunch planner plan.launch map:=1`


For the grid based planners:
`roslaunch planner plan.launch map:=2`


# Nodes
* `prm_planner`: searches the probabilistic road map using theta*
* `draw_cont_map` : draw the map in continuous space
* `gird_planner` : path planning on grid and constructs map as a 2D occupancy grid



# Probabilisitc Roadmap
## Theta* Shortest Path
<p align="center">
  <img src= "media/thetastar.png" width="350" height="500">
</p>
