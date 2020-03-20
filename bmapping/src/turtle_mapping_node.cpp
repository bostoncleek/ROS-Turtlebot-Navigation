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
#include "tsim/PoseError.h"


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
using rigid2d::normalize_angle_PI;


using bmapping::LaserProperties;
using bmapping::LaserScanner;
using bmapping::ScanAlignment;
using bmapping::ParticleFilter;
using bmapping::GridMapper;


static std::string left_wheel_joint, right_wheel_joint;    // joint names
static double left, right;                                 // wheel angular positions for odometry
static double pf_left, pf_right;                         // wheel angular positions for SLAM
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

  ros::Publisher slam_path_pub = node_handle.advertise<nav_msgs::Path>("slam_path", 10);
  ros::Publisher odom_path_pub = node_handle.advertise<nav_msgs::Path>("odom_path", 10);
  ros::Publisher gazebo_path_pub = node_handle.advertise<nav_msgs::Path>("gazebo_path", 10);
  ros::Publisher map_pub = node_handle.advertise<nav_msgs::OccupancyGrid>("map", 10);
  ros::Publisher odom_pub = node_handle.advertise<nav_msgs::Odometry>("odom", 1);

  ros::Publisher odom_error_pub = node_handle.advertise<tsim::PoseError>("odom_error", 1);
  ros::Publisher slam_error_pub = node_handle.advertise<tsim::PoseError>("slam_error", 1);

  /////////////////////////////////////////////////////////////////////////////

  // frame IDs
  // robots chassis parameters
  std::string map_frame_id, odom_frame_id, body_frame_id;
  double wheel_base = 0.0, wheel_radius = 0.0;

  // lidar specs
  double beam_min = 0.0, beam_max = 0.0, beam_delta = 0.0;
  double range_min = 0.0, range_max = 0.0;

  // scan likelihood parameters
  double z_hit = 0.0, z_short = 0.0, z_max = 0.0, z_rand = 0.0, sigma_hit = 0.0;

  // particle filter parameters
  int num_particles = 0, num_samples_mode = 0;

  double srr = 0.0, srt = 0.0, str = 0.0, stt = 0.0;
  double motion_noise_theta = 0.0, motion_noise_x = 0.0, motion_noise_y = 0.0;
  double sample_range_theta = 0.0, sample_range_x = 0.0, sample_range_y = 0.0;
  double scan_likelihood_min = 0.0, scan_likelihood_max = 0.0;
  double pose_likelihood_min = 0.0, pose_likelihood_max = 0.0;

  // occupancy grid parameters
  double map_min = 0.0, map_max = 0.0, map_resolution = 0.0;


  nh.getParam("left_wheel_joint", left_wheel_joint);
  nh.getParam("right_wheel_joint", right_wheel_joint);

  nh.getParam("map_frame_id", map_frame_id);
  nh.getParam("odom_frame_id", odom_frame_id);
  nh.getParam("body_frame_id", body_frame_id);

  nh.getParam("beam_min", beam_min);
  nh.getParam("beam_max", beam_max);
  nh.getParam("beam_delta", beam_delta);
  nh.getParam("range_min", range_min);
  nh.getParam("range_max", range_max);

  nh.getParam("z_hit", z_hit);
  nh.getParam("z_short", z_short);
  nh.getParam("z_max", z_max);
  nh.getParam("z_rand", z_rand);
  nh.getParam("sigma_hit", sigma_hit);

  nh.getParam("num_particles", num_particles);
  nh.getParam("num_samples_mode", num_samples_mode);

  nh.getParam("srr", srr);
  nh.getParam("srt", srt);
  nh.getParam("str", str);
  nh.getParam("stt", stt);

  nh.getParam("motion_noise_theta", motion_noise_theta);
  nh.getParam("motion_noise_x", motion_noise_x);
  nh.getParam("motion_noise_y", motion_noise_y);

  nh.getParam("sample_range_theta", sample_range_theta);
  nh.getParam("sample_range_x", sample_range_x);
  nh.getParam("sample_range_y", sample_range_y);

  nh.getParam("scan_likelihood_min", scan_likelihood_min);
  nh.getParam("scan_likelihood_max", scan_likelihood_max);

  nh.getParam("pose_likelihood_min", pose_likelihood_min);
  nh.getParam("pose_likelihood_max", pose_likelihood_max);

  nh.getParam("map_min", map_min);
  nh.getParam("map_max", map_max);
  nh.getParam("map_resolution", map_resolution);


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

  ROS_INFO("z_hit %f", z_hit);
  ROS_INFO("z_short %f", z_short);
  ROS_INFO("z_max %f", z_max);
  ROS_INFO("z_rand %f", z_rand);
  ROS_INFO("sigma_hit %f", sigma_hit);

  ROS_INFO("num_particles %d", num_particles);
  ROS_INFO("num_samples_mode %d", num_samples_mode);

  ROS_INFO("srr %f", srr);
  ROS_INFO("srt %f", srt);
  ROS_INFO("str %f", str);
  ROS_INFO("stt %f", stt);

  ROS_INFO("motion_noise_theta %.15f", motion_noise_theta);
  ROS_INFO("motion_noise_x %.15f", motion_noise_x);
  ROS_INFO("motion_noise_y %.15f", motion_noise_y);

  ROS_INFO("sample_range_theta %.15f", sample_range_theta);
  ROS_INFO("sample_range_x %.15f", sample_range_x);
  ROS_INFO("sample_range_y %.15f", sample_range_y);

  ROS_INFO("scan_likelihood_min %f", scan_likelihood_min);
  ROS_INFO("scan_likelihood_max %f", scan_likelihood_max);

  ROS_INFO("pose_likelihood_min %f", pose_likelihood_min);
  ROS_INFO("pose_likelihood_max %f", pose_likelihood_max);

  ROS_INFO("map_min %f", map_min);
  ROS_INFO("map_max %f", map_max);
  ROS_INFO("map_resolution %f", map_resolution);

  ROS_INFO("map_frame_id %s", map_frame_id.c_str());
  ROS_INFO("odom_frame_id %s", odom_frame_id.c_str());
  ROS_INFO("body_frame_id %s", body_frame_id.c_str());

  ROS_INFO("left_wheel_joint %s", left_wheel_joint.c_str());
  ROS_INFO("right_wheel_joint %s", right_wheel_joint.c_str());

  ROS_INFO("wheel_base %f", wheel_base);
  ROS_INFO("wheel_radius %f", wheel_radius);

  ROS_INFO("Successfully launched RBPF SLAM node");

  /////////////////////////////////////////////////////////////////////////////

  tf2_ros::TransformBroadcaster slam_broadcaster;
  tf2_ros::TransformBroadcaster odom_broadcaster;

  wheel_odom_flag = false;
  scan_update = false;


  // Assume pose starts at (0,0,0)
  // for odometry
  rigid2d::Pose pose;
  pose.theta = 0;
  pose.x = 0;
  pose.y = 0;

  // odometry poses for SLAM pose likelihood
  rigid2d::Pose cur_odom;
  rigid2d::Pose prev_odom;
  cur_odom = prev_odom = pose;

  // Assume pose starts at (0,0,0)
  // SLAM
  Transform2D robot_pose;


  // diff drive model for odometry
  rigid2d::DiffDrive drive(pose, wheel_base, wheel_radius);
  // diff drive model for SLAM
  rigid2d::DiffDrive pf_drive(pose, wheel_base, wheel_radius);


  /////////////////////////////////////////////////////////////////////////////

  // transform robot to lidar
  Transform2D Trs;

  // lidar properties
  LaserProperties props(beam_min, beam_max, beam_delta, range_min, range_max,
                              z_hit, z_short, z_max, z_rand, sigma_hit);

  // grid mapping
  // set map as square
  GridMapper grid(map_resolution, map_min, map_max, map_min, map_max, props, Trs);

  // pcl ICP
  ScanAlignment aligner(props, Trs);


  // particle filter
  ParticleFilter pf(num_particles, num_samples_mode,
                    srr, srt, str, stt,
                    motion_noise_theta, motion_noise_x, motion_noise_y,
                    sample_range_theta, sample_range_x, sample_range_y,
                    scan_likelihood_min, scan_likelihood_max,
                    pose_likelihood_min, pose_likelihood_max,
                    aligner, robot_pose, grid);


  // path from odometry
  nav_msgs::Path odom_path;

  // path from SLAM
  nav_msgs::Path pf_path;

  // path from gazebo
  nav_msgs::Path gazebo_path;

  // error in pose
  tsim::PoseError odom_error_msg;
  tsim::PoseError slam_error_msg;

  /////////////////////////////////////////////////////////////////////////////

  // the map
  std::vector<int8_t> map;

  geometry_msgs::Pose map_pose;
  map_pose.position.x = map_min;
  map_pose.position.y = map_min;
  map_pose.position.z = 0;
  map_pose.orientation.x = 0;
  map_pose.orientation.y = 0;
  map_pose.orientation.z = 0;
  map_pose.orientation.w = 1;


  nav_msgs::OccupancyGrid map_msg;
  map_msg.header.frame_id = map_frame_id;
  map_msg.info.resolution = map_resolution;
  map_msg.info.width = static_cast<int> ((map_max - map_min) / map_resolution);
  map_msg.info.height = static_cast<int> ((map_max - map_min) / map_resolution);;

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

      pf_left = left;
      pf_right = right;

      // update ekf with odometry and sensor measurements
      if (scan_update)
      {
        // do the slam
        pf_drive.updateOdometry(pf_left, pf_right);
        cur_odom = pf_drive.pose();
        rigid2d::WheelVelocities vel = pf_drive.wheelVelocities();
        rigid2d::Twist2D vb = pf_drive.wheelsToTwist(vel);

        pf.SLAM(scan, vb, cur_odom, prev_odom);

        map_msg.header.stamp = ros::Time::now();
        map_msg.info.map_load_time = ros::Time::now();
        map_msg.data = map;
        pf.newMap(map);

        prev_odom = cur_odom;

        scan_update = false;
      }

      wheel_odom_flag = false;
    }

    /////////////////////////////////////////////////////////////////////////////

    // braodcast transform from map to odom
    // transform from map to robot
    // Transform2D Tmr = robot_pose;
    Transform2D Tmr = pf.getRobotState();;


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

    // broadcast transform from odom to base_link
    // publish twist in odom message
    rigid2d::WheelVelocities vel = drive.wheelVelocities();
    rigid2d::Twist2D vb = drive.wheelsToTwist(vel);


    // convert yaw to Quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, pose.theta);
    geometry_msgs::Quaternion odom_quat;
    odom_quat = tf2::toMsg(q);


    // broadcast transform between odom and body
    geometry_msgs::TransformStamped odom_tf;
    odom_tf.header.stamp = ros::Time::now();
    odom_tf.header.frame_id = odom_frame_id;
    odom_tf.child_frame_id = body_frame_id;

    odom_tf.transform.translation.x = pose.x;
    odom_tf.transform.translation.y = pose.y;
    odom_tf.transform.translation.z = 0.0;
    odom_tf.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_tf);


    // publish odom message over ros
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = odom_frame_id;
    odom.child_frame_id = body_frame_id;


    // pose in odom frame
    odom.pose.pose.position.x = pose.x;
    odom.pose.pose.position.y = pose.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    // velocity in body frame
    odom.twist.twist.linear.x = vb.vx;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = vb.w;

    odom_pub.publish(odom);


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

    // publish the map
    map_pub.publish(map_msg);

    /////////////////////////////////////////////////////////////////////////////

    // ground truth robot heading
    tf2::Quaternion gazebo_robot_quat(gazebo_robot_pose.pose.orientation.x,
                               gazebo_robot_pose.pose.orientation.y,
                               gazebo_robot_pose.pose.orientation.z,
                               gazebo_robot_pose.pose.orientation.w);

    tf2::Matrix3x3 mat(gazebo_robot_quat);
    auto roll = 0.0, pitch = 0.0 , yaw = 0.0;
    mat.getRPY(roll, pitch, yaw);


    // odometry error
    odom_error_msg.x_error = gazebo_robot_pose.pose.position.x - pose.x;
    odom_error_msg.y_error = gazebo_robot_pose.pose.position.y - pose.y;

    odom_error_msg.theta_error = normalize_angle_PI(normalize_angle_PI(yaw) - \
                                                     normalize_angle_PI(pose.theta));

    // slam error
    TransformData2D Trd_mr = Tmr.displacement();

    slam_error_msg.x_error = gazebo_robot_pose.pose.position.x - Trd_mr.x;
    slam_error_msg.y_error = gazebo_robot_pose.pose.position.y - Trd_mr.y;

    slam_error_msg.theta_error = normalize_angle_PI(normalize_angle_PI(yaw) - \
                                                     normalize_angle_PI(Trd_mr.theta));

    odom_error_pub.publish(odom_error_msg);
    slam_error_pub.publish(slam_error_msg);
  }

  return 0;
}






















//
