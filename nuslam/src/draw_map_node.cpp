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
#include <visualization_msgs/MarkerArray.h>

#include <vector>
#include <string>
#include <iostream>

#include "nuslam/TurtleMap.h"

static bool map_update;
static std::vector<double> cx;
static std::vector<double> cy;
static std::vector<double> r;
static std::string frame_id;





void mapCallback(const nuslam::TurtleMap::ConstPtr &msg)
{
  cx = msg->cx;
  cy = msg->cy;
  r = msg->r;
  frame_id = msg->header.frame_id;
  map_update = true;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "draw_map");
  ros::NodeHandle nh("~");
  ros::NodeHandle node_handle;

  map_update = false;

  ros::Subscriber scan_sub = node_handle.subscribe("landmarks", 1, mapCallback);
  ros::Publisher marker_pub = node_handle.advertise<visualization_msgs::MarkerArray>("map", 100);



  while(node_handle.ok())
  {
    ros::spinOnce();

    if (map_update)
    {
      visualization_msgs::MarkerArray marker_array;
      marker_array.markers.resize(r.size());

      for(unsigned int i = 0; i < r.size(); i++)
      {
        marker_array.markers[i].header.frame_id = frame_id;
        marker_array.markers[i].header.stamp = ros::Time::now();
        marker_array.markers[i].lifetime = ros::Duration(1.0/ 5.0); // 1/5th sec
        marker_array.markers[i].ns = "marker";
        marker_array.markers[i].id = i;

        marker_array.markers[i].type = visualization_msgs::Marker::CYLINDER;
        marker_array.markers[i].action = visualization_msgs::Marker::ADD;

        marker_array.markers[i].pose.position.x = cx.at(i);
        marker_array.markers[i].pose.position.y = cy.at(i);
        marker_array.markers[i].pose.position.z = 0.0;

        marker_array.markers[i].pose.orientation.x = 0.0;
        marker_array.markers[i].pose.orientation.y = 0.0;
        marker_array.markers[i].pose.orientation.z = 0.0;
        marker_array.markers[i].pose.orientation.w = 1.0;

        marker_array.markers[i].scale.x = 2.0 * r.at(i);
        marker_array.markers[i].scale.y = 2.0 * r.at(i);
        marker_array.markers[i].scale.z = 0.1;

        marker_array.markers[i].color.r = 0.0f;
        marker_array.markers[i].color.g = 1.0f;
        marker_array.markers[i].color.b = 0.0f;
        marker_array.markers[i].color.a = 1.0f;
      }
      marker_pub.publish(marker_array);



      map_update = false;
    }


  }
  return 0;
}















// end file
