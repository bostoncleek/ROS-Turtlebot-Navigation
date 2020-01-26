/// \file
/// \brief  publishes odometry messages
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
#include <tf/transform_broadcaster.h>

#include <string>
#include <vector>
#include <iostream>


#include "rigid2d/diff_drive.hpp"


// global variables
std::string left_wheel_joint, right_wheel_joint;    // joint names
double left, right;                                // wheel angular positions
bool message;

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

  // ROS_INFO("Left wheel %f", left);
  // ROS_INFO("Right wheel %f", right);

  left = msg->position.at(left_idx);
  right = msg->position.at(right_idx);

  message = true;
}

// TODO: change to tf2



int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometer");
  ros::NodeHandle node_handle("~");

  ros::Subscriber joint_sub = node_handle.subscribe("/joint_states", 1, jointStatesCallback);
  ros::Publisher odom_pub = node_handle.advertise<nav_msgs::Odometry>("odom", 1);
  tf::TransformBroadcaster odom_broadcaster;


  std::string odom_frame_id, body_frame_id;
  double wheel_base, wheel_radius;


  node_handle.param<std::string>("odom_frame_id", odom_frame_id, "odom");
  node_handle.param<std::string>("body_frame_id", body_frame_id, "base_link");
  node_handle.param<std::string>("left_wheel_joint", left_wheel_joint, "left_wheel_axle");
  node_handle.param<std::string>("right_wheel_joint", right_wheel_joint, "right_wheel_axle");


  node_handle.getParam("/wheel_base", wheel_base);
  node_handle.getParam("/wheel_radius", wheel_radius);

  ROS_INFO("wheel_base: %f", wheel_base);
  ROS_INFO("wheel_radius: %f", wheel_radius);


  ROS_INFO("Successfully launched odometer node.");


  // check if message has been recieved
  message = false;

  rigid2d::Pose pose;
  pose.theta = 0;
  pose.x = 0;
  pose.y = 0;


  // diff drive model
  rigid2d::DiffDrive drive(pose, wheel_base, wheel_radius);

  // timing
  // int frequency = 60;
  // ros::Rate loop_rate(frequency);

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();


  while(node_handle.ok())
  {
    ros::spinOnce();
    current_time = ros::Time::now();

    if (message)
    {
      ROS_INFO("Left wheel %f", left);
      ROS_INFO("Right wheel %f", right);
      // ROS_INFO("Right wheel %f", right);


      // update odom
      drive.updateOdometry(left, right);

      // pose relative to odom frame
      pose = drive.pose();
      // std::cout << "------------------" << std::endl;
      // std::cout << "odometer" << std::endl;
      // std::cout << pose.theta << " " << pose.x << " " << pose.y << std::endl;
      // std::cout << "------------------" << std::endl;

      // body twist
      rigid2d::WheelVelocities vel = drive.wheelVelocities();
      rigid2d::Twist2D vb = drive.wheelsToTwist(vel);

      // quaternion from yaw
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose.theta);
      // std::cout << odom_quat << "\n";


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

      message = false;
    }

    last_time = current_time;
    // loop_rate.sleep();
  }

  return 0;
}












// end file
