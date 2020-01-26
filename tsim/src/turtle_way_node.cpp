/// \file
/// \brief  compose twist to move turtle in pentagon
///
/// \author Boston Cleek
/// \date 1/25/20
///
/// PUBLISHES:
///   cmd_vel (Twist): linear and angular velocity controls
///   pose_error (PoseError): error in turtles pose
/// SUBSCRIBES:
///   turtlesim/Pose (Pose): pose of turtle


#include <ros/ros.h>
#include <ros/console.h>
#include <turtlesim/Pose.h>
#include <turtlesim/SetPen.h>
#include <turtlesim/TeleportAbsolute.h>
#include <geometry_msgs/Twist.h>


#include <string>
#include <vector>
#include <iostream>

#include <rigid2d/rigid2d.hpp>
#include <rigid2d/diff_drive.hpp>
#include <rigid2d/waypoints.hpp>
#include "tsim/PoseError.h"



// global variables
// turtlesim::Pose tsim_pose;
rigid2d::Pose tsim_pose;



/// \brief Starts robot at first waypoint and places down pen
/// \param node_handle - node handle
/// \param x - x component of frist waypoint
/// \param y - y component of frist waypoint
void initTrajectory(ros::NodeHandle &node_handle, float x, float y)
{
  ros::ServiceClient pen_client = node_handle.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");
  turtlesim::SetPen pen_srv;

  ros::ServiceClient tele_client = node_handle.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");
  turtlesim::TeleportAbsolute tele_srv;


  // turn off pen
  pen_srv.request.r = 0;
  pen_srv.request.g = 0;
  pen_srv.request.b = 0;
  pen_srv.request.width = 0;
  pen_srv.request.off = 1;

  pen_client.call(pen_srv);

  // teleport
  tele_srv.request.x = x;
  tele_srv.request.y = y;
  tele_srv.request.theta = 0;

  tele_client.call(tele_srv);


  // turn pen on
  pen_srv.request.r = 255;
  pen_srv.request.g = 255;
  pen_srv.request.b = 255;
  pen_srv.request.width = 2;
  pen_srv.request.off = 0;

  pen_client.call(pen_srv);

  ROS_INFO("Teleported to Start");
}


/// \brief callback for updating turtle's pose
/// \param msg - contains pose
void poseCallback(const turtlesim::Pose::ConstPtr &msg)
{
  tsim_pose.theta = msg->theta;
  tsim_pose.x = msg->x;
  tsim_pose.y = msg->y;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtle_way");
  ros::NodeHandle node_handle;

  // waypoints
  std::vector<float> waypoint_x;
  std::vector<float> waypoint_y;

  // controls
  rigid2d::Twist2D cmd;
  double rot_vel = 0.5;
  double trans_vel = 0.5;


  // loop rate
  int frequency = 60;

  // pose for diff drive model
  rigid2d::Pose dd_pose;


  ros::Subscriber pose_sub = node_handle.subscribe("pose", 1, poseCallback);
  ros::Publisher vel_pub = node_handle.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Publisher error_pub = node_handle.advertise<tsim::PoseError>("pose_error", 1);


  node_handle.getParam("x_component", waypoint_x);
  node_handle.getParam("y_component", waypoint_y);

  // wait for servies to be availible
  ros::service::waitForService("/turtle1/set_pen", -1);
  ros::service::waitForService("/turtle1/teleport_absolute", -1);

  ROS_INFO("Successfully launched turtle_way node.");

  // start at 1st waypoint
  initTrajectory(node_handle, waypoint_x.at(0), waypoint_y.at(0));

  dd_pose.theta = 0;
  dd_pose.x = waypoint_x.at(0);
  dd_pose.y = waypoint_y.at(0);


  std::vector<rigid2d::Vector2D> pts;
  for(unsigned int i = 0; i < waypoint_x.size(); i++)
  {
    rigid2d::Vector2D v;
    v.x = waypoint_x.at(i);
    v.y = waypoint_y.at(i);
    pts.push_back(v);
  }


  rigid2d::Waypoints waypts(pts, rot_vel, trans_vel);
  rigid2d::DiffDrive drive;

  drive.reset(dd_pose);

  ros::Rate loop_rate(frequency);

  while(node_handle.ok())
  {
    ros::spinOnce();

    // update pose
    dd_pose = drive.pose();


    // update twist
    cmd = waypts.nextWaypoint(dd_pose);


    // scale twist based on frequency
    rigid2d::Twist2D scaled;
    scaled.vx = cmd.vx* 1 / frequency;
    scaled.w = cmd.w * 1 / frequency;
    // propogate kinematics
    drive.feedforward(scaled);


    // publish cmd_vel to turtle
    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = cmd.vx;
    twist_msg.linear.y = 0;
    twist_msg.linear.z = 0;
    twist_msg.angular.x = 0;
    twist_msg.angular.y = 0;
    twist_msg.angular.z = cmd.w;

    //  error
    tsim::PoseError error_msg;
    error_msg.x_error = std::abs(dd_pose.x - tsim_pose.x);
    error_msg.y_error = std::abs(dd_pose.y - tsim_pose.y);
    error_msg.theta_error = std::abs(std::abs(dd_pose.theta) - std::abs(tsim_pose.theta));


    vel_pub.publish(twist_msg);
    error_pub.publish(error_msg);

    loop_rate.sleep();
  }
  return 0;
}












// end file
