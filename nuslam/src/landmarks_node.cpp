/// \file
/// \brief Detects and extracts circular features in laser scan
///
/// \author Boston Cleek
/// \date 2/27/20
///
/// PARAMETERS:
///   frame_id - frame the circles are in
/// PUBLISHES:
///   landmarks (nuslam::TurtleMap): center and radius of all circles detected
/// SUBSCRIBES:
///   scan (sensor_msgs/LaserScan): Lidar scan

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>

#include <vector>
#include <string>
#include <iostream>

#include <rigid2d/rigid2d.hpp>
#include "nuslam/TurtleMap.h"
#include "nuslam/landmarks.hpp"

using nuslam::LaserProperties;
using nuslam::Landmarks;

using rigid2d::Vector2D;
using rigid2d::deg2rad;


static std::vector<float> scan;         // lidar scan
static bool scan_update;                // scan update flag


/// \brief Update the scan
/// \param msg -lidar scan
void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  scan = msg->ranges;
  scan_update = true;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "landmarks");
  ros::NodeHandle nh("~");
  ros::NodeHandle node_handle;

  ros::Subscriber scan_sub = node_handle.subscribe("scan", 1, scanCallback);
  ros::Publisher circle_pub = node_handle.advertise<nuslam::TurtleMap>("landmarks", 1);

  // frame id
  std::string frame_id;
  nh.getParam("frame_id", frame_id);

  ROS_INFO("frame_id %s\n", frame_id.c_str());

  ROS_INFO("Successfully launched landmarks node");

  scan_update = false;

  // lidar properties
  double beam_min = 0.0, beam_max = deg2rad(360.0);
  double beam_delta = deg2rad(1.0);
  double range_min = 0.12, range_max = 3.5;
  LaserProperties props(beam_min, beam_max, beam_delta, range_min, range_max);


  // landmark classifier
  double epsilon = 0.075;
  Landmarks landmarks(props, epsilon);


  while(node_handle.ok())
  {
    ros::spinOnce();

    if (scan_update)
    {
      // find features in new scan
      landmarks.featureDetection(scan);

      // new map
      nuslam::TurtleMap map;
      map.header.frame_id = frame_id;
      map.header.stamp = ros::Time::now();


      // std::cout << "Number of circles: " << landmarks.lm.size() << std::endl;

      for(unsigned int i = 0; i < landmarks.lm.size(); i++)
      {
        map.cx.push_back(landmarks.lm.at(i).x_hat);
        map.cy.push_back(landmarks.lm.at(i).y_hat);
        map.r.push_back(landmarks.lm.at(i).radius);

      }
      circle_pub.publish(map);
      scan_update = false;

    }
  }
  return 0;
}


// end file
