/// \file
/// \brief publishes odometry messages and broadcasts a tf from odom to base link frame
///
/// \author Boston Cleek
/// \date 3/16/20
///
/// PUBLISHES:
///   odom (nav_msgs/Odometry): Pose of robot in odom frame and twist in body frame
///
/// SUBSCRIBES:
///   joint_states (sensor_msgs/JointState): angular wheel positions
///
/// SERVICES:
///


#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <iostream>
#include <memory>
#include <cstdlib>
#include <vector>
#include <time.h>
#include <cmath>
#include <Eigen/Core>

#include <rigid2d/rigid2d.hpp>
#include <rigid2d/diff_drive.hpp>
#include "bmapping/cloud_alignment.hpp"
#include "bmapping/sensor_model.hpp"
#include "bmapping/grid_mapper.hpp"
#include "bmapping/particle_filter.hpp"


using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector2d;
using Eigen::Vector3d;

using rigid2d::Twist2D;
using rigid2d::Vector2D;
using rigid2d::Pose;
using rigid2d::Transform2D;
using rigid2d::TransformData2D;
using rigid2d::deg2rad;

using bmapping::LaserProperties;
using bmapping::LaserScanner;
using bmapping::ScanAlignment;
using bmapping::ParticleFilter;
using bmapping::GridMapper;


static std::string left_wheel_joint, right_wheel_joint;    // joint names
static double left, right;                                 // wheel angular positions for odometry
static double ekf_left, ekf_right;                         // wheel angular positions for SLAM
static bool wheel_odom_flag;                               // odometry update

static std::vector<float> scan;
static bool scan_update;

static geometry_msgs::PoseStamped gazebo_robot_pose;



void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  // std::cout << "scan callback" << std::endl;
  scan = msg->ranges;
  scan_update = true;
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

  ekf_left += left;
  ekf_right += right;


  wheel_odom_flag = true;
}


void modelCallBack(const gazebo_msgs::ModelStates::ConstPtr& model_data)
{
  // store names of all items in gazebo
  std::vector<std::string> names = model_data->name;

  // index of robot
  int robot_index = 0;

  // find diff_drive robot
  int ctr = 0;
  for(const auto &item : names)
  {
    // check for robot
    if (item == "diff_drive")
    {
      robot_index = ctr;
    }

    ctr++;
  } // end loop


  // pose of robot
  gazebo_robot_pose.header.stamp = ros::Time::now();
  gazebo_robot_pose.pose.position = model_data->pose[robot_index].position;
  gazebo_robot_pose.pose.orientation = model_data->pose[robot_index].orientation;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "slam");
  ros::NodeHandle nh("~");
  ros::NodeHandle node_handle;

  /////////////////////////////////////////////////////////////////////////////

  ros::Subscriber scan_sub = node_handle.subscribe("scan", 1, scanCallback);
  ros::Subscriber joint_sub = node_handle.subscribe("joint_states", 1, jointStatesCallback);
  ros::Subscriber model_sub = nh.subscribe("/gazebo/model_states", 1, modelCallBack);

  ros::Publisher slam_path_pub = node_handle.advertise<nav_msgs::Path>("slam_path", 1);
  ros::Publisher odom_path_pub = node_handle.advertise<nav_msgs::Path>("odom_path", 1);
  ros::Publisher gazebo_path_pub = node_handle.advertise<nav_msgs::Path>("gazebo_path", 1);
  ros::Publisher map_pub = node_handle.advertise<nav_msgs::OccupancyGrid>("map", 1);

  /////////////////////////////////////////////////////////////////////////////

  std::string map_frame_id, odom_frame_id, marker_frame_id;
  auto wheel_base = 0.0, wheel_radius = 0.0;

  // lidar specs
  double beam_min = 0.0, beam_max = 0.0, beam_delta = 0.0;
  double range_min = 0.0, range_max = 0.0;

  nh.getParam("left_wheel_joint", left_wheel_joint);
  nh.getParam("right_wheel_joint", right_wheel_joint);

  nh.getParam("map_frame_id", map_frame_id);
  nh.getParam("odom_frame_id", odom_frame_id);
  nh.getParam("marker_frame_id", marker_frame_id);

  nh.getParam("beam_min", beam_min);
  nh.getParam("beam_max", beam_max);
  nh.getParam("beam_delta", beam_delta);
  nh.getParam("range_min", range_min);
  nh.getParam("range_max", range_max);

  node_handle.getParam("/wheel_base", wheel_base);
  node_handle.getParam("/wheel_radius", wheel_radius);

  beam_min = deg2rad(beam_min);
  beam_max = deg2rad(beam_max);
  beam_delta = deg2rad(beam_delta);

  ROS_INFO("beam_min %f", beam_min);
  ROS_INFO("beam_max %f", beam_max);
  ROS_INFO("beam_delta %f", beam_delta);
  ROS_INFO("range_min %f", range_min);
  ROS_INFO("range_max %f", range_max);


  ROS_WARN("map_frame_id %s", map_frame_id.c_str());
  ROS_WARN("odom_frame_id %s", odom_frame_id.c_str());
  ROS_WARN("marker_frame_id %s", marker_frame_id.c_str());

  ROS_INFO("left_wheel_joint %s", left_wheel_joint.c_str());
  ROS_INFO("right_wheel_joint %s", right_wheel_joint.c_str());

  ROS_INFO("wheel_base %f", wheel_base);
  ROS_INFO("wheel_radius %f", wheel_radius);

  ROS_INFO("Successfully launched RBPF SLAM node");

  /////////////////////////////////////////////////////////////////////////////

  tf2_ros::TransformBroadcaster slam_broadcaster;

  wheel_odom_flag = false;
  scan_update = false;


  // Assume pose starts at (0,0,0)
  // for odometry
  rigid2d::Pose pose;
  pose.theta = 0;
  pose.x = 0;
  pose.y = 0;

  // Assume pose starts at (0,0,0)
  // SLAM
  Transform2D robot_pose;


  // diff drive model for odometry
  rigid2d::DiffDrive drive(pose, wheel_base, wheel_radius);
  // diff drive model for SLAM
  rigid2d::DiffDrive ekf_drive(pose, wheel_base, wheel_radius);

  /////////////////////////////////////////////////////////////////////////////

  // transform robot to lidar
  Transform2D Trs;

  // lidar properties
  double z_hit = 0.95;
  double z_short = 0.00; // not used in model
  double z_max = 1.0 / 25.0;
  double z_rand = 1.0 - z_hit - z_max;
  double sigma_hit = 0.5;  // 0.5 also works okay, consider resolution of map

  LaserProperties props(beam_min, beam_max, beam_delta, range_min, range_max,
                              z_hit, z_short, z_max, z_rand, sigma_hit);

  // grid mapping
  GridMapper grid(0.05, -2, 2, -2, 2, props, Trs);

  // pcl ICP
  ScanAlignment aligner(props, Trs);


  // particle filter
  TransformData2D T2d_pose = robot_pose.displacement();
  Pose init_pose_pf;
  init_pose_pf.theta = T2d_pose.theta;
  init_pose_pf.x = T2d_pose.x;
  init_pose_pf.y = T2d_pose.y;

  ParticleFilter pf(100, -1, aligner, init_pose_pf, grid);


  // path from odometry
  nav_msgs::Path odom_path;

  // path from SLAM
  nav_msgs::Path pf_path;

  // path from gazebo
  nav_msgs::Path gazebo_path;

  /////////////////////////////////////////////////////////////////////////////

  // the map
  std::vector<int8_t> map;

  geometry_msgs::Pose map_pose;
  map_pose.position.x = -2;
  map_pose.position.y = -2;
  map_pose.position.z = 0;
  map_pose.orientation.x = 0;
  map_pose.orientation.y = 0;
  map_pose.orientation.z = 0;
  map_pose.orientation.w = 1;


  nav_msgs::OccupancyGrid map_msg;
  map_msg.header.frame_id = map_frame_id;
  map_msg.info.resolution = 0.05;
  map_msg.info.width = 80;
  map_msg.info.height = 80;

  map_msg.info.origin = map_pose;

  /////////////////////////////////////////////////////////////////////////////

  while(node_handle.ok())
  {
    ros::spinOnce();

    /////////////////////////////////////////////////////////////////////////////
    if (wheel_odom_flag)
    {
      // most recent odom update
      drive.updateOdometry(left, right);
      pose = drive.pose();

      // update ekf with odometry and sensor measurements
      if (scan_update)
      {
        // do the slam
        // grid.integrateScan(scan, robot_pose);
        // grid.gridMap(map);
        //
        // std::cout << "P(z|m,x) " << std::endl;
        // std::cout << grid.likelihoodFieldModel(scan, robot_pose) << std::endl;
        //
        // // find transform between scans
        // Transform2D Tpcl;
        // // initial guess
        // Transform2D T_init;
        //
        // aligner.pclICPWrapper(Tpcl, T_init, scan);
        // robot_pose = robot_pose * Tpcl;

        ekf_drive.updateOdometry(ekf_left, ekf_right);
        rigid2d::WheelVelocities vel = ekf_drive.wheelVelocities();
        rigid2d::Twist2D vb = ekf_drive.wheelsToTwist(vel);

        pf.SLAM(scan, vb);
        robot_pose = pf.getRobotState();
        pf.newMap(map);

        map_msg.header.stamp = ros::Time::now();
        map_msg.info.map_load_time = ros::Time::now();
        map_msg.data = map;


        ekf_left = 0.0;
        ekf_right = 0.0;

        scan_update = false;
      }

      wheel_odom_flag = false;
    }

    /////////////////////////////////////////////////////////////////////////////

    // braodcast transform from map to odom
    // transform from map to robot
    Transform2D Tmr = robot_pose;/* = ekf.getRobotState();*/

    // transform from odom to robot
    Vector2D vor(pose.x, pose.y);
    Transform2D Tor(vor, pose.theta);

    // transform from robot to odom
    Transform2D Tro = Tor.inv();

    // now we can get transform from map to odom
    Transform2D Tmo = Tmr *  Tro;
    TransformData2D pose_map_odom = Tmo.displacement();


    tf2::Quaternion q_mo;
    q_mo.setRPY(0, 0, pose_map_odom.theta);
    geometry_msgs::Quaternion quat_mo;
    quat_mo = tf2::toMsg(q_mo);


    // broadcast transform between map and odom
    geometry_msgs::TransformStamped tf_mo;
    tf_mo.header.stamp = ros::Time::now();
    tf_mo.header.frame_id = map_frame_id;
    tf_mo.child_frame_id = odom_frame_id;

    tf_mo.transform.translation.x = pose_map_odom.x;
    tf_mo.transform.translation.y = pose_map_odom.y;
    tf_mo.transform.translation.z = 0.0;
    tf_mo.transform.rotation = quat_mo;

    slam_broadcaster.sendTransform(tf_mo);

    /////////////////////////////////////////////////////////////////////////////

    // path from SLAM
    geometry_msgs::PoseStamped slam_pose;
    TransformData2D pose_map_robot = Tmr.displacement();

    tf2::Quaternion q_mr;
    q_mr.setRPY(0, 0, pose_map_robot.theta);
    geometry_msgs::Quaternion quat_mr;
    quat_mr = tf2::toMsg(q_mr);

    slam_pose.header.stamp = ros::Time::now();
    slam_pose.pose.position.x = pose_map_robot.x;
    slam_pose.pose.position.y = pose_map_robot.y;
    slam_pose.pose.orientation = quat_mr;

    pf_path.header.stamp = ros::Time::now();
    pf_path.header.frame_id = map_frame_id;
    pf_path.poses.push_back(slam_pose);

    slam_path_pub.publish(pf_path);

    /////////////////////////////////////////////////////////////////////////////

    // path from odom
    geometry_msgs::PoseStamped odom_pose;

    tf2::Quaternion q_or;
    q_or.setRPY(0, 0, pose.theta);
    geometry_msgs::Quaternion quat_or;
    quat_or = tf2::toMsg(q_or);

    odom_pose.header.stamp = ros::Time::now();
    odom_pose.pose.position.x = pose.x;
    odom_pose.pose.position.y = pose.y;
    odom_pose.pose.orientation = quat_or;

    odom_path.header.stamp = ros::Time::now();
    odom_path.header.frame_id = map_frame_id;
    odom_path.poses.push_back(odom_pose);

    odom_path_pub.publish(odom_path);

    /////////////////////////////////////////////////////////////////////////////

    // path from gazebo
    gazebo_path.header.stamp = ros::Time::now();
    gazebo_path.header.frame_id = map_frame_id;
    gazebo_path.poses.push_back(gazebo_robot_pose);

    gazebo_path_pub.publish(gazebo_path);

    /////////////////////////////////////////////////////////////////////////////

    map_pub.publish(map_msg);
  }

  return 0;
}






















//
