/// \file
/// \brief
///
/// \author Boston Cleek
/// \date 3/5/20
///
/// PUBLISHES:
///
/// SUBSCRIBES:
///
/// SERVICES:


#include <ros/ros.h>
#include <ros/console.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <string>
#include <vector>
#include <iostream>

#include <rigid2d/rigid2d.hpp>
#include "nuslam/landmarks.hpp"
#include "nuslam/TurtleMap.h"

using rigid2d::Vector2D;
using rigid2d::Transform2D;
using rigid2d::TransformData2D;

using nuslam::pointDistance;

static double radius_circles;
static double radius;
static std::string frame_id;
static geometry_msgs::Pose robot_pose;
static nuslam::TurtleMap map;
static bool model_update;


void modelCallBack(const gazebo_msgs::ModelStates::ConstPtr& model_data)
{
  // store names of all items in gazebo
  std::vector<std::string> names = model_data->name;

  // indices of cylinders
  std::vector<int> cylinder_indices;
  // index of robot
  int robot_index = 0;


  // find cylinders and diff_drive robot
  int ctr = 0;
  for(const auto &item : names)
  {
    // check for a c
    if (item[0] == 'c')
    {
      cylinder_indices.push_back(ctr);
    }

    // check for robot
    if (item == "diff_drive")
    {
      robot_index = ctr;
    }

    ctr++;
  } // end loop


  // pose of robot
  robot_pose.position = model_data->pose[robot_index].position;
  robot_pose.orientation = model_data->pose[robot_index].orientation;

  tf2::Quaternion robot_quat(model_data->pose[robot_index].orientation.x,
                             model_data->pose[robot_index].orientation.y,
                             model_data->pose[robot_index].orientation.z,
                             model_data->pose[robot_index].orientation.w);

  tf2::Matrix3x3 mat(robot_quat);
  auto roll = 0.0, pitch = 0.0 , yaw = 0.0;
  mat.getRPY(roll, pitch, yaw);



  // x/y location of robot
  Vector2D p1;
  p1.x = robot_pose.position.x;
  p1.y = robot_pose.position.y;
  // map to robot
  Transform2D Tmr(p1, yaw);


  // clear previous map
  map.cx.clear();
  map.cy.clear();
  map.r.clear();


  // find cylinders within radius around robot
  for(const auto index : cylinder_indices)
  {
    Vector2D p2;
    p2.x = model_data->pose[index].position.x;
    p2.y = model_data->pose[index].position.y;

    // map to landmark
    Transform2D Tml(p2);

    // robot to map
    Transform2D Trm = Tmr.inv();

    // robot to landmark
    Transform2D Trl = Trm * Tml;

    // position of landmark in frame of robot
    TransformData2D plr = Trl.displacement();

    // note: use relative position of landmarks
    if (pointDistance(p1, p2) <= radius)
    {
      map.cx.push_back(plr.x);
      map.cy.push_back(plr.y);

      // hard code radius of circle
      map.r.push_back(radius_circles);
    }

    // outside the radius set values to nan
    else
    {
      map.cx.push_back(NAN);
      map.cy.push_back(NAN);

      // hard code radius of circle
      map.r.push_back(radius_circles);
    }
  }


  model_update = true;
}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "analysis");
  ros::NodeHandle nh("~");
  ros::NodeHandle node_handle;

  ros::Subscriber scan_sub = nh.subscribe("/gazebo/model_states", 1, modelCallBack);
  ros::Publisher pose_pub = node_handle.advertise<geometry_msgs::Pose>("robot_pose", 1);
  ros::Publisher circle_pub = node_handle.advertise<nuslam::TurtleMap>("landmarks", 1);

  model_update = false;


  // radius from robot to landmarks
  // which landmarks are in sight
  // detect all circles within this radius
  nh.getParam("radius", radius);
  nh.getParam("frame_id", frame_id);

  ROS_WARN("search radius %f\n", radius);
  ROS_WARN("frame_id %s\n", frame_id.c_str());

  ROS_INFO("Successfully launched analysis node");

  // hard code radius of circles to publish in map
  radius_circles = 0.05;


  while(node_handle.ok())
  {
    ros::spinOnce();

    if (model_update)
    {
      map.header.frame_id = frame_id;
      map.header.stamp = ros::Time::now();

      pose_pub.publish(robot_pose);
      circle_pub.publish(map);

      model_update = false;
    }
  }

return 0;
}



























// end file
