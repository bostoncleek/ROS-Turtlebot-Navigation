### Boston Cleek

# Overview:
This packages is apart of a larger ROS autonomous framework for the Turtlebot. This package contains implementations of D* Light on a grid, Theta* and Potential Fields in continuous space.


# How to run
D* Light: <br/>
`roslaunch planner plan.launch plan_type:=1`

Theta*: <br/>
`roslaunch planner plan.launch plan_type:=2`

Potential Field: <br/>
`roslaunch planner plan.launch plan_type:=3`



# Nodes
* `grid_planner`: D* light and constructs map as a 2D occupancy grid
* `prm_planner`: constructs the probabilistic road map and finds shortest path using Theta*
* `potential_field_planner`: path planning using attractive and repulsive fields
* `draw_cont_map`: draws the map in continuous space



# D* Light
The implementation contains two internal representations of the grid map represented as a 2D occupancy grid. The first in the complete map with all the obstacles. The second map is initialized empty and obstacles are added based on the range of the visibility of the robot.

This algorithm plans from the goal to the starting cell. Originally the shortest path is the straight line to the goal. As the path front moves toward the goal obstacles come into view, the map updates, and the algorithm replans. The heuristic used here is euclidean distance.

The green cells represent the current shortest path. The red squares are the cells that are updated during the replanning stage after a map update.

## 8 cell visibility
<p align="center">
  <img src= "media/dstar8cells.gif" width="350" height="500">
</p>


## 1 cell visibility
<p align="center">
  <img src= "media/dstar1cells.gif" width="350" height="500">
</p>


# Theta* on Probabilisitc Roadmap
A probabilistic roadmap is constructed by sampling points randomly in free space and connecting them the their nearest neighbors. Theta* is similar to A* the only difference is Theta* optimizes the shortest path by skipping over nodes that can be connected by a straight line.

The roadmap show in greed was constructed using 200 nodes where each node is connected to 10 of its nearest neighbors. The shortest path is shown in blue. The heuristic used here is euclidean distance.

<p align="center">
  <img src= "media/thetastar.png" width="350" height="500">
</p>


# Potential Fields
An attractive gradient pulls the path front to the goal and a repulsive gradient repels the path front away from obstacles. The next position in the path front is obtained using one step of gradient descent. The descent direction is composed by summating the attractive and repulsive gradients and normalizing the sum using the L2-norm.

The green line is the path, the blue dots are the start/goal positions, and the red lines are the obstacle edges and bounds of the map.

<p align="center">
  <img src= "media/potentialfield.gif" width="350" height="500">
</p>


# Algorithms

## D* Light
`Koenig, Sven, and Maxim Likhachev. "Fast replanning for navigation in unknown terrain." IEEE Transactions on Robotics 21.3 (2005): 354-363.`

<p align="center">
  <img src= "media/dstaralgo.png" width="350" height="500">
</p>

## Theta*
The following algorithm is A* the only change to Theta* is the updateVertex() function.

`Nash, Alex, et al. "Theta^*: Any-angle path planning on grids." AAAI. Vol. 7. 2007.`

<p align="center">
  <img src= "media/thetastaruv.png" width="350" height="300">
</p>

<p align="center">
  <img src= "media/astaralgo.png" width="450" height="450">
</p>

## Potential Field

<p align="center">
  <img src= "media/potfieldalgo.png" width="500" height="350">
</p>
