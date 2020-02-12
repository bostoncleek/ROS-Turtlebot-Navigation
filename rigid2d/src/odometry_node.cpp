/// \file
/// \brief publishes odometry messages and broadcasts a tf from odom to base link frame
///
/// \author Boston Cleek
/// \date 1/23/20
///
/// PUBLISHES:
///   odom (nav_msgs/Odometry): Pose of robot in odom frame and twist in body frame
///
/// SUBSCRIBES:
///   joint_states (sensor_msgs/JointState): angular wheel positions


#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>
#include <vector>
#include <iostream>
#include <exception>

#include "rigid2d/diff_drive.hpp"
#include "rigid2d/set_pose.h"

// global variables
static std::string left_wheel_joint, right_wheel_joint;    // joint names
static double left, right;                                 // wheel angular positions
// static bool message;                                       // callback flag

static rigid2d::Pose pose_srv;                             // pose set by srv
static bool srv_active;                                    // set pose srv activated



/// \brief service sets the pose of the robot
/// \param req - service request pose
/// \param res - service pose result
bool setPoseService(rigid2d::set_pose::Request &req,
                    rigid2d::set_pose::Response &res)
{
  // the requested pose
  pose_srv.theta = req.theta;
  pose_srv.x = req.x;
  pose_srv.y = req.y;

  ROS_INFO("pose %f %f %f", pose_srv.theta, pose_srv.x, pose_srv.y);
  // set response
  res.set_pose_state = true;

  // service flag has been activated
  srv_active = true;

  ROS_INFO("Set Pose service activated");

  return true;
}


/// \brief updates the wheel encoder angles
/// \param msg - contains the encoder readings and joint names
void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<std::string> names = msg->name;
  std::vector<std::string>::iterator iter;
  int left_idx, right_idx;

  iter = std::find(names.begin(), names.end(), left_wheel_joint);
  left_idx = std::distance(names.begin(), iter);

  iter = std::find(names.begin(), names.end(), right_wheel_joint);
  right_idx = std::distance(names.begin(), iter);

  if (left_idx > 1)
  {
    throw std::invalid_argument("Left wheel index not found in Odometer.");
  }

  if (right_idx > 1)
  {
    throw std::invalid_argument("Right wheel index not found in Odometer.");
  }


  // ROS_INFO("left index: %d", left_idx);
  // ROS_INFO("right index: %d", right_idx);


  left = msg->position.at(left_idx);
  right = msg->position.at(right_idx);

  // message = true;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometer");
  ros::NodeHandle nh("~");
  ros::NodeHandle node_handle;

  ros::Subscriber joint_sub = node_handle.subscribe("joint_states", 1, jointStatesCallback);
  ros::Publisher odom_pub = node_handle.advertise<nav_msgs::Odometry>("odom", 1);

  ros::ServiceServer ps_server = node_handle.advertiseService("set_pose", setPoseService);


  tf2_ros::TransformBroadcaster odom_broadcaster;

  std::string odom_frame_id, body_frame_id;
  double wheel_base = 0.0, wheel_radius = 0.0;


  // node_handle.getParam("odom_frame_id", odom_frame_id);
  // node_handle.getParam("body_frame_id", body_frame_id);

  nh.getParam("left_wheel_joint", left_wheel_joint);
  nh.getParam("right_wheel_joint", right_wheel_joint);

  nh.getParam("odom_frame_id", odom_frame_id);
  nh.getParam("body_frame_id", body_frame_id);

  node_handle.getParam("/wheel_base", wheel_base);
  node_handle.getParam("/wheel_radius", wheel_radius);


  ROS_WARN("odom_frame_id %s", odom_frame_id.c_str());
  ROS_WARN("body_frame_id %s", body_frame_id.c_str());

  ROS_INFO("left_wheel_joint %s", left_wheel_joint.c_str());
  ROS_INFO("right_wheel_joint %s", right_wheel_joint.c_str());

  ROS_INFO("wheel_base %f", wheel_base);
  ROS_INFO("wheel_radius %f", wheel_radius);


  ROS_INFO("Successfully launched odometer node.");


  // check if message has been recieved
  // message = false;

  // service has been requested
  srv_active = false;

  // Assume pose starts at (0,0,0)
  rigid2d::Pose pose;
  pose.theta = 0;
  pose.x = 0;
  pose.y = 0;


  // diff drive model
  rigid2d::DiffDrive drive(pose, wheel_base, wheel_radius);

  // timing
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  while(node_handle.ok())
  {
    ros::spinOnce();
    current_time = ros::Time::now();


    // set pose service flag
    if (srv_active)
    {
      drive.reset(pose_srv);

      srv_active = false;
    }


    // joint states call back flag
    // if (message)
    // {
    // update odom
    drive.updateOdometry(left, right);

    // pose relative to odom frame
    pose = drive.pose();
    // std::cout << "------------------" << std::endl;
    // std::cout << "odometer" << std::endl;
    // std::cout << pose.theta << " " << pose.x << " " << pose.y << std::endl;
    // std::cout << "------------------" << std::endl;

    // ROS_INFO("pose %f", pose.theta);


    // body twist
    rigid2d::WheelVelocities vel = drive.wheelVelocities();
    rigid2d::Twist2D vb = drive.wheelsToTwist(vel);


    // convert yaw to Quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, pose.theta);
    geometry_msgs::Quaternion odom_quat;
    odom_quat = tf2::toMsg(q);


    // broadcast transform between odom and body
    geometry_msgs::TransformStamped odom_tf;
    odom_tf.header.stamp = current_time;
    odom_tf.header.frame_id = odom_frame_id;
    odom_tf.child_frame_id = body_frame_id;

    odom_tf.transform.translation.x = pose.x;
    odom_tf.transform.translation.y = pose.y;
    odom_tf.transform.translation.z = 0.0;
    odom_tf.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_tf);


    // publish odom message over ros
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = odom_frame_id;
    odom.child_frame_id = body_frame_id;


    // pose in odom frame
    odom.pose.pose.position.x = pose.x;
    odom.pose.pose.position.y = pose.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    // velocity in body frame
    odom.twist.twist.linear.x = vb.vx;
    odom.twist.twist.linear.y = vb.vy; // should always be 0
    odom.twist.twist.angular.z = vb.w;

    odom_pub.publish(odom);

    //   message = false;
    // }



    last_time = current_time;
  }
  return 0;
}












// end file
