/// \file
/// \brief Path planning on grid and constructs map as a 2D occupancy grid
///
/// \author Boston Cleek
/// \date 4/15/20
///
/// PUBLISHES:
/// map (nav_msgs::OccupancyGrid>): free/occupied/buffer zone for planning


#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <geometry_msgs/Pose.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include "planner/grid_map.hpp"


using rigid2d::Vector2D;




int main(int argc, char** argv)
{
  ros::init(argc, argv, "grid_planner");
  ros::NodeHandle nh("~");
  ros::NodeHandle node_handle;

  ros::Publisher map_pub = node_handle.advertise<nav_msgs::OccupancyGrid>("map", 1);


  auto obs_resolution = 0.0;              // resolution of obstacle coordinates
  auto grid_resolution = 0.0;             // resolution of grid cell (cell side length)
  std::string frame_id;                   // frame of grid
  XmlRpc::XmlRpcValue map_bound;          // boundaries of map
  XmlRpc::XmlRpcValue obstacles;          // triple nested vector for obstacle coordinates
  auto bounding_radius = 0.0;             // bounding radius around robot for collisions

  nh.getParam("frame_id", frame_id);
  nh.getParam("bounding_radius", bounding_radius);
  nh.getParam("grid_resolution", grid_resolution);


  nh.getParam("resolution", obs_resolution);
  nh.getParam("bounds", map_bound);
  nh.getParam("obstacles", obstacles);


  const auto xmin = static_cast<double>(map_bound[0][0]) * obs_resolution;
  const auto xmax = static_cast<double>(map_bound[0][1]) * obs_resolution;

  const auto ymin = static_cast<double>(map_bound[1][0]) * obs_resolution;
  const auto ymax = static_cast<double>(map_bound[1][1]) * obs_resolution;


  ROS_INFO("Successfully launched grid_planner node");


  /////////////////////////////////////////////////////////////////////////////

  // load obstacles
  planner::obstacle_map obs_map;

  for(auto i = 0; i < obstacles.size(); i++)
  {
    planner::polygon poly;
    for(auto j = 0; j < obstacles[i].size(); j++)
    {
      Vector2D p;
      p.x = static_cast<double> (obstacles[i][j][0]) * obs_resolution;
      p.y = static_cast<double> (obstacles[i][j][1]) * obs_resolution;
      poly.push_back(p);
    }
    obs_map.push_back(poly);
  }


  // set up the grid
  planner::GridMap grid(xmin, xmax,
                        ymin, ymax,
                        grid_resolution,
                        bounding_radius,
                        obs_map);

  // rviz representation of the grid
  std::vector<int8_t> map;

  // pose of grid
  geometry_msgs::Pose map_pose;
  map_pose.position.x = 0.0;
  map_pose.position.y = 0.0;
  map_pose.position.z = 0.0;

  map_pose.orientation.x = 0.0;
  map_pose.orientation.y = 0.0;
  map_pose.orientation.z = 0.0;
  map_pose.orientation.w = 1.0;


  nav_msgs::OccupancyGrid map_msg;
  map_msg.header.frame_id = frame_id;
  map_msg.info.resolution = grid_resolution;
  map_msg.info.width = planner::gridSize(xmin, xmax, grid_resolution);
  map_msg.info.height = planner::gridSize(ymin, ymax, grid_resolution);

  map_msg.info.origin = map_pose;


  // std::cout << planner::gridSize(ymin, ymax, resolution) << std::endl;
  // std::cout << "size test: " << std::round((ymax - ymin) / resolution) << std::endl;
  // std::cout << "size test: " << ((ymax - ymin) / resolution) << std::endl;


  grid.labelCells();
  grid.getGrid(map);



  while(node_handle.ok())
  {
    ros::spinOnce();


    map_msg.header.stamp = ros::Time::now();
    map_msg.info.map_load_time = ros::Time::now();
    map_msg.data = map;
    map_pub.publish(map_msg);


  }




  return 0;
}



// end file
