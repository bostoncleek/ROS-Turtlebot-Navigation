/// \file
/// \brief
///
/// \author Boston Cleek
/// \date 4/15/20
///
/// PUBLISHES:
///


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


  auto resolution = 0.0;                  // resolution of map coordinates
  std::string frame_id;                   // frame of grid
  XmlRpc::XmlRpcValue map_bound;          // boundaries of map
  XmlRpc::XmlRpcValue obstacles;          // triple nested vector for obstacle coordinates
  auto bounding_radius = 0.0;             // bounding radius around robot for collisions

  nh.getParam("frame_id", frame_id);
  nh.getParam("bounding_radius", bounding_radius);

  nh.getParam("resolution", resolution);
  nh.getParam("bounds", map_bound);
  nh.getParam("obstacles", obstacles);


  const auto xmin = static_cast<double>(map_bound[0][0]) * resolution;
  const auto xmax = static_cast<double>(map_bound[0][1]) * resolution;

  const auto ymin = static_cast<double>(map_bound[1][0]) * resolution;
  const auto ymax = static_cast<double>(map_bound[1][1]) * resolution;


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
      p.x = static_cast<double> (obstacles[i][j][0]) * resolution;
      p.y = static_cast<double> (obstacles[i][j][1]) * resolution;
      poly.push_back(p);
    }
    obs_map.push_back(poly);
  }


  // set up the grid
  planner::GridMap grid(xmin, xmax,
                        ymin, ymax,
                        resolution,
                        bounding_radius,
                        obs_map);

  std::cout << grid.grid2World(0, 1) << std::endl;

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
  map_msg.info.resolution = resolution;
  map_msg.info.width = static_cast<unsigned int> (std::ceil((xmax - xmin) / resolution));
  map_msg.info.height = static_cast<unsigned int> (std::ceil((ymax - ymin) / resolution));

  map_msg.info.origin = map_pose;


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
