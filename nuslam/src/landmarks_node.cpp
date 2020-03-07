/// \file
/// \brief
///
/// \author Boston Cleek
/// \date 2/27/20
///
/// PUBLISHES:
///
/// SUBSCRIBES:
///
/// SERVICES:

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


static std::vector<float> scan;
static bool scan_update;




void pointCloud(const std::vector<Vector2D> &end_points,
                sensor_msgs::PointCloud &point_cloud)
{
  for(const auto &point : end_points)
  {
    geometry_msgs::Point32 p;
    p.x = point.x;
    p.y = point.y;
    p.z = 0.05;

    point_cloud.points.push_back(p);
  }
}


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


  ros::Publisher cloud_pub = node_handle.advertise<sensor_msgs::PointCloud>("point_cloud_test", 100);


  // frame id
  std::string frame_id;
  nh.getParam("frame_id", frame_id);

  ROS_WARN("frame_id %s\n", frame_id.c_str());

  ROS_INFO("Successfully launched landmarks node");

  scan_update = false;

  // lidar properties
  double beam_min = 0.0, beam_max = deg2rad(360.0);
  double beam_delta = deg2rad(1.0);
  double range_min = 0.12, range_max = 3.5;
  LaserProperties props(beam_min, beam_max, beam_delta, range_min, range_max);


  // landmark classifier
  double epsilon = 0.05;
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




        // std::cout << "radius: " << landmarks.lm.at(i).radius <<
        //              " cx: " << landmarks.lm.at(i).x_hat <<
        //              " cy: " << landmarks.lm.at(i).y_hat << std::endl;


      }
      circle_pub.publish(map);
      scan_update = false;



      // point cloud for testing
      // std::vector<Vector2D> end_points;
      // landmarks.laserEndPoints(end_points, scan);

      // sensor_msgs::PointCloud point_cloud;
      // point_cloud.header.stamp = ros::Time::now();
      // point_cloud.header.frame_id = "base_scan";
      // pointCloud(end_points, point_cloud);

      // if (!landmarks.lm.empty())
      // {
      //   for(unsigned int i = 0; i < landmarks.lm.size(); i++)
      //   {
      //     pointCloud(landmarks.lm.at(i).points, point_cloud);
      //     cloud_pub.publish(point_cloud);
      //   }
      // }

    }
  }
  return 0;
}









// end file
