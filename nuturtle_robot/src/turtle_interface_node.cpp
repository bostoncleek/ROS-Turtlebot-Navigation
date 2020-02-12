/// \file
/// \brief Low level interface for commanding turtlebot manevuers
///
/// \author Boston Cleek
/// \date 2/5/20
///
/// PUBLISHES:
///   wheel_cmd (nuturtlebot/WheelCommands): commanded wheel speed -44 to 44
///   joint_states (sensor_msgs/JointState): position and velocity of each wheel
/// SUBSCRIBES:
///   cmd_vel (geometry_msgs/Twist): Commanded body twist
///   sensor_data (nuturtlebot/SensorData): wheel encoder sesnor data



#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

#include <string>
#include <iostream>
#include <cmath>
#include <algorithm>


#include <rigid2d/rigid2d.hpp>
#include <rigid2d/diff_drive.hpp>
#include <nuturtlebot/WheelCommands.h>
#include <nuturtlebot/SensorData.h>



// global variables
static rigid2d::Twist2D cmd;                                // body cmd_vel
static bool twist_message;                                  // twist callback flag

static double left, right;                                 //  encoder ticks
static bool sensor_message;                                // sensor callback flag



/// \brief updates the body twist of the robot
/// \param msg - contains the velocities of the body frame
void twistCallBack(const geometry_msgs::Twist::ConstPtr &msg)
{
  cmd.w = msg->angular.z;
  cmd.vx = msg->linear.x;
  cmd.vy = 0.0;


  // ROS_INFO("twistCallback");
  twist_message = true;
}



/// \brief updates wheel encoder readings
/// \param msg - contains sensor information
void sensorCallBack(const nuturtlebot::SensorData::ConstPtr &msg)
{
  left = msg->left_encoder;
  right = msg->right_encoder;

  sensor_message = true;
}



int main(int argc, char** argv)
{

  ros::init(argc, argv, "turtle_interface");
  ros::NodeHandle nh("~");
  ros::NodeHandle node_handle;

  ros::Subscriber vel_sub = node_handle.subscribe("cmd_vel", 1, twistCallBack);
  ros::Subscriber sensor_sub = node_handle.subscribe("/sensor_data", 1, sensorCallBack);

  ros::Publisher wheel_pub = node_handle.advertise<nuturtlebot::WheelCommands>("/wheel_cmd", 1);
  ros::Publisher joint_pub = node_handle.advertise<sensor_msgs::JointState>("joint_states", 1);


  double max_rot = 0.0, max_trans = 0.0;
  double wheel_base = 0.0, wheel_radius = 0.0;
  double max_rot_motor = 0.0, encoder_ticks_per_rev = 0.0, max_motor_power = 0.0;

  std::string left_wheel_joint, right_wheel_joint;

  node_handle.getParam("/max_rot", max_rot);
  node_handle.getParam("/max_trans", max_trans);
  node_handle.getParam("/max_rot_motor", max_rot_motor);
  node_handle.getParam("/encoder_ticks_per_rev", encoder_ticks_per_rev);
  node_handle.getParam("/max_motor_power", max_motor_power);


  node_handle.getParam("/wheel_base", wheel_base);
  node_handle.getParam("/wheel_radius", wheel_radius);


  nh.getParam("left_wheel_joint", left_wheel_joint);
  nh.getParam("right_wheel_joint", right_wheel_joint);


  ROS_INFO("left_wheel_joint %s", left_wheel_joint.c_str());
  ROS_INFO("right_wheel_joint %s", right_wheel_joint.c_str());
  ROS_INFO("max_motor_power %f", max_motor_power);


  ROS_INFO("Successfully launched turtle_interface node");



  // init pose of robot (0,0,0)
  rigid2d::Pose ps;

  // model the robot
  rigid2d::DiffDrive drive(ps, wheel_base, wheel_radius);


  // timing
   ros::Time current_time, last_time;
   current_time = ros::Time::now();
   last_time = ros::Time::now();



  while(node_handle.ok())
  {
    ros::spinOnce();
    current_time = ros::Time::now();
    last_time = current_time;


    if (twist_message)
    {
      // cap velocities
      cmd.w = std::clamp(cmd.w, -max_rot, max_rot);
      cmd.vx = std::clamp(cmd.vx, -max_trans, max_trans);


      // compose wheel velocities from twist
      rigid2d::WheelVelocities wheel_vel = drive.twistToWheels(cmd);


      // cap wheel velocities
      wheel_vel.ul = std::clamp(wheel_vel.ul, -max_rot_motor, max_rot_motor);
      wheel_vel.ur = std::clamp(wheel_vel.ur, -max_rot_motor, max_rot_motor);


      // scale wheel velocities -44 -> 44
      nuturtlebot::WheelCommands wheel_command;
      wheel_command.left_velocity = std::round((max_motor_power / max_rot_motor) * wheel_vel.ul);
      wheel_command.right_velocity = std::round((max_motor_power / max_rot_motor) * wheel_vel.ur);


      // ROS_INFO("left wheel cmd %f", wheel_vel.ul);
      // ROS_INFO("right wheel cmd %f", wheel_vel.ur);
      //
      // ROS_INFO("left wheel cmd %d", wheel_command.left_velocity);
      // ROS_INFO("right wheel cmd %d", wheel_command.right_velocity);

      twist_message = false;

      wheel_pub.publish(wheel_command);
    }


    if (sensor_message)
    {

      // convert endcoder ticks to wheel angles
      double left_angle = rigid2d::normalize_angle_PI((2*rigid2d::PI / encoder_ticks_per_rev) * left);
      double right_angle = rigid2d::normalize_angle_PI((2*rigid2d::PI / encoder_ticks_per_rev) * right);


      // update odometry to get wheel velocities
      rigid2d::WheelVelocities wheel_vel =  drive.updateOdometry(left_angle, right_angle);



      sensor_msgs::JointState joint_state;
      joint_state.header.stamp = current_time;

      joint_state.name.push_back(left_wheel_joint);
      joint_state.name.push_back(right_wheel_joint);

      joint_state.position.push_back(left_angle);
      joint_state.position.push_back(right_angle);

      joint_state.velocity.push_back(wheel_vel.ul);
      joint_state.velocity.push_back(wheel_vel.ur);



      // ROS_WARN("left wheel pos %f", left_angle);
      // ROS_WARN("right wheel pos %f", right_angle);
      //
      // ROS_INFO("left wheel vel %f", wheel_vel.ul);
      // ROS_INFO("right wheel vel %f", wheel_vel.ur);


      sensor_message = false;

      joint_pub.publish(joint_state);
    }

  }

  return 0;
}


















// end file
