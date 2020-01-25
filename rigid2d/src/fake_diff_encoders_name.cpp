/// \file
/// \brief  publishes odometry messages
///
/// \author Boston Cleek
/// \date 1/24/20
///
/// PUBLISHES:
/// joint_states (sensor_msgs/JointState): angular wheel positions
///
/// SUBSCRIBES:
/// cmd_vel (geometry_msgs/Twist): twist in body frame


#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

#include <string>
#include <vector>
#include <iostream>

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"



// global variables
std::string left_wheel_joint, right_wheel_joint;     // joint names
rigid2d::Twist2D cmd;


/// \brief updates the body twist of the robot
/// \param msg - contains the encoder readings and joint names
void twistCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  cmd.w = msg->angular.z;
  cmd.vx = msg->linear.x;
  cmd.vy = msg->linear.y; // should be 0

  // std::cout << "------------------" << std::endl;
  // std::cout << cmd.w << " " << cmd.vx << " " << cmd.vy << std::endl;
  // std::cout << "------------------" << std::endl;
}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "fake_diff_encoders");
  ros::NodeHandle node_handle("~");

  ros::Subscriber twist_sub = node_handle.subscribe("/turtle1/cmd_vel", 1, twistCallback);
  ros::Publisher joint_pub = node_handle.advertise<sensor_msgs::JointState>("/joint_states", 1);

  double wheel_base, wheel_radius;

  node_handle.param<std::string>("left_wheel_joint", left_wheel_joint, "left_wheel_axle");
  node_handle.param<std::string>("right_wheel_joint", right_wheel_joint, "right_wheel_axle");

  node_handle.getParam("/wheel_base", wheel_base);
  node_handle.getParam("/wheel_radius", wheel_radius);

  ROS_INFO("wheel_base: %f", wheel_base);
  ROS_INFO("wheel_radius: %f", wheel_radius);

  ROS_INFO("Successfully launched fake_diff_encoders node.");


  rigid2d::Pose pose;
  pose.theta = 0;
  pose.x = 0;
  pose.y = 0;


  // diff drive model
  rigid2d::DiffDrive drive(pose, wheel_base, wheel_radius);


  while(node_handle.ok())
  {
    ros::spinOnce();

    // ROS_INFO("left %f", encoders.left);

    // propogate kinematics
    drive.feedforward(cmd);

    rigid2d::Pose pose = drive.pose();
    std::cout << "------------------" << std::endl;
    std::cout << pose.theta << " " << pose.x << " " << pose.y << std::endl;
    std::cout << "------------------" << std::endl;


    // wheel encoder readings
    rigid2d::WheelEncoders encoders = drive.getEncoders();

    sensor_msgs::JointState joint_state;
    joint_state.name.push_back(left_wheel_joint);
    joint_state.name.push_back(right_wheel_joint);

    joint_state.position.push_back(encoders.left);
    joint_state.position.push_back(encoders.right);

    // ROS_INFO("left %f", encoders.left);
    // ROS_INFO("right %f", encoders.right);

    cmd.w = 0.0;
    cmd.vx = 0.0;
    cmd.vy = 0.0;


    joint_pub.publish(joint_state);
  }

  return 0;
}













// end file
