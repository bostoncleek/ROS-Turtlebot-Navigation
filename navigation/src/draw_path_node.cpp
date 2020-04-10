/// \file
/// \brief
///
/// \author Boston Cleek
/// \date 3/29/20
///
/// PARAMETERS:

/// PUBLISHES:

/// SUBSCRIBES:

#include <ros/ros.h>
#include <ros/console.h>

#include "navigation/road_map.hpp"

using navigation::RoadMap;
using navigation::FeatureMap;
using rigid2d::Vector2D;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "draw_path");
  ros::NodeHandle nh("~");
  ros::NodeHandle node_handle;


  RoadMap path(-2.0, 2.0, -2.0, 2.0, 0.01, 0.01, 2, 4);

  FeatureMap f_map;
  f_map.cx.push_back(0.0);
  f_map.cy.push_back(0.0);
  f_map.r.push_back(1.0);


  Vector2D s(0.0, 0.0);
  Vector2D g(1.0, 1.0);

  path.constructRoadMap(f_map, s, g);
  path.printRoadMap();




  return 0;
}










// end file
