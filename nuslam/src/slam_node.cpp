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
///
/// SERVICES:
///   set_pose (set_pose) - sets the pose of the robot


#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>
#include <vector>
#include <iostream>
#include <exception>
#include <Eigen/Core>


#include <rigid2d/diff_drive.hpp>
#include "nuslam/filter.hpp"
#include "rigid2d/set_pose.h"
#include "nuslam/TurtleMap.h"


using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using rigid2d::Twist2D;
using rigid2d::Vector2D;
using rigid2d::Pose;
using rigid2d::Transform2D;
using rigid2d::TransformData2D;

static std::string left_wheel_joint, right_wheel_joint;    // joint names
static double left, right;                                 // wheel angular positions
// static bool message;                                       // callback flag

// static rigid2d::Pose pose_srv;                             // pose set by srv
// static bool srv_active;                                    // set pose srv activated


static std::vector<Vector2D> meas;
static bool map_flag;



void mapCallBack(const nuslam::TurtleMap::ConstPtr &map_msg)
{
  // clear previous measurement
  meas.clear();
  meas.reserve(map_msg->r.size());
  for(unsigned int i = 0; i < map_msg->r.size(); i++)
  {
    Vector2D m;
    m.x = map_msg->cx.at(i);
    m.y = map_msg->cy.at(i);
    meas.push_back(m);
  }

  map_flag = true;
}


/// \brief service sets the pose of the robot
/// \param req - service request pose
/// \param res - service pose result
// bool setPoseService(rigid2d::set_pose::Request &req,
//                     rigid2d::set_pose::Response &res)
// {
//   // the requested pose
//   pose_srv.theta = req.theta;
//   pose_srv.x = req.x;
//   pose_srv.y = req.y;
//
//   ROS_INFO("pose %f %f %f", pose_srv.theta, pose_srv.x, pose_srv.y);
//   // set response
//   res.set_pose_state = true;
//
//   // service flag has been activated
//   srv_active = true;
//
//   ROS_INFO("Set Pose service activated");
//
//   return true;
// }


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
  ros::init(argc, argv, "slam");
  ros::NodeHandle nh("~");
  ros::NodeHandle node_handle;

  /////////////////////////////////////////////////////////////////////////////

  ros::Subscriber joint_sub = node_handle.subscribe("joint_states", 1, jointStatesCallback);
  ros::Subscriber map_sub = node_handle.subscribe("landmarks", 1, mapCallBack);
  ros::Publisher slam_path_pub = node_handle.advertise<nav_msgs::Path>("slam_path", 1);
  ros::Publisher odom_path_pub = node_handle.advertise<nav_msgs::Path>("odom_path", 1);
  ros::Publisher marker_pub = node_handle.advertise<visualization_msgs::MarkerArray>("slam_map", 100);
  // ros::ServiceServer ps_server = node_handle.advertiseService("set_pose", setPoseService);

  /////////////////////////////////////////////////////////////////////////////
  // load in real landmark data
  std::vector<double> lm_x;
  std::vector<double> lm_y;
  std::vector<int> lm_id;

  nh.getParam("x", lm_x);
  nh.getParam("y", lm_y);
  nh.getParam("id", lm_id);

  std::vector<Vector3d> landmarks;
  for(unsigned int i = 0; i < lm_id.size(); i++)
  {
    Vector3d lmvec;
    lmvec << lm_x.at(i), lm_y.at(i), lm_id.at(i);
    // std::cout << lmvec << std::endl;
    landmarks.push_back(lmvec);
  }
  /////////////////////////////////////////////////////////////////////////////

  std::string map_frame_id, odom_frame_id, marker_frame_id;
  auto wheel_base = 0.0, wheel_radius = 0.0;

  nh.getParam("left_wheel_joint", left_wheel_joint);
  nh.getParam("right_wheel_joint", right_wheel_joint);

  nh.getParam("map_frame_id", map_frame_id);
  nh.getParam("odom_frame_id", odom_frame_id);
  nh.getParam("marker_frame_id", marker_frame_id);


  node_handle.getParam("/wheel_base", wheel_base);
  node_handle.getParam("/wheel_radius", wheel_radius);


  ROS_WARN("map_frame_id %s", map_frame_id.c_str());
  ROS_WARN("odom_frame_id %s", odom_frame_id.c_str());
  ROS_WARN("marker_frame_id %s", marker_frame_id.c_str());

  ROS_INFO("left_wheel_joint %s", left_wheel_joint.c_str());
  ROS_INFO("right_wheel_joint %s", right_wheel_joint.c_str());

  ROS_INFO("wheel_base %f", wheel_base);
  ROS_INFO("wheel_radius %f", wheel_radius);

  ROS_INFO("Successfully launched slam node");

  /////////////////////////////////////////////////////////////////////////////

  tf2_ros::TransformBroadcaster odom_broadcaster;
  map_flag = false;
  // srv_active = false;


  // Assume pose starts at (0,0,0)
  // for odometry
  rigid2d::Pose pose;
  pose.theta = 0;
  pose.x = 0;
  pose.y = 0;


  // diff drive model
  rigid2d::DiffDrive drive(pose, wheel_base, wheel_radius);

  // path from odometry
  nav_msgs::Path odom_path;


  int n = 12;
  nuslam::EKF ekf(n);
  ekf.setKnownLandamrks(landmarks);

  // path from SLAM
  nav_msgs::Path ekf_path;

  /////////////////////////////////////////////////////////////////////////////


  // int frequency = 10;
  // ros::Rate loop_rate(frequency);

  // Turn noise in gazebo back on

  while(node_handle.ok())
  {
    ros::spinOnce();

    // set pose service flag
    // if (srv_active)
    // {
    //   drive.reset(pose_srv);
    //
    //   srv_active = false;
    // }

    /////////////////////////////////////////////////////////////////////////////

    drive.updateOdometry(left, right);
    pose = drive.pose();
    rigid2d::WheelVelocities vel = drive.wheelVelocities();
    rigid2d::Twist2D vb = drive.wheelsToTwist(vel);

    /////////////////////////////////////////////////////////////////////////////

    if (map_flag)
    {
      ekf.knownCorrespondenceSLAM(meas, vb);
      map_flag = false;
    }


    /////////////////////////////////////////////////////////////////////////////

    // braodcast transform from map to odom
    // transform from map to robot
    Transform2D Tmr = ekf.getRobotState();

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

    odom_broadcaster.sendTransform(tf_mo);

    /////////////////////////////////////////////////////////////////////////////

    // path from SLAM
    geometry_msgs::PoseStamped slam_pose;
    TransformData2D pose_map_robot = Tmr.displacement();

    tf2::Quaternion q_mr;
    q_mr.setRPY(0, 0, pose_map_robot.theta);
    geometry_msgs::Quaternion quat_mr;
    quat_mr = tf2::toMsg(q_mr);

    slam_pose.pose.position.x = pose_map_robot.x;
    slam_pose.pose.position.y = pose_map_robot.y;
    slam_pose.pose.orientation = quat_mr;

    ekf_path.header.stamp = ros::Time::now();
    ekf_path.header.frame_id = map_frame_id;
    ekf_path.poses.push_back(slam_pose);

    slam_path_pub.publish(ekf_path);

    /////////////////////////////////////////////////////////////////////////////

    // path from odom
    geometry_msgs::PoseStamped odom_pose;

    tf2::Quaternion q_or;
    q_or.setRPY(0, 0, pose.theta);
    geometry_msgs::Quaternion quat_or;
    quat_or = tf2::toMsg(q_or);

    odom_pose.pose.position.x = pose.x;
    odom_pose.pose.position.y = pose.y;
    odom_pose.pose.orientation = quat_or;

    odom_path.header.stamp = ros::Time::now();
    odom_path.header.frame_id = map_frame_id;
    odom_path.poses.push_back(odom_pose);

    odom_path_pub.publish(odom_path);

    /////////////////////////////////////////////////////////////////////////////

    // marker array of landmark estimates from the ekf filter
    std::vector<Vector2D> map;
    ekf.getMap(map);

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(map.size());

    for(unsigned int i = 0; i < map.size(); i++)
    {
      marker_array.markers[i].header.frame_id = marker_frame_id;
      marker_array.markers[i].header.stamp = ros::Time::now();
      marker_array.markers[i].lifetime = ros::Duration(1.0/ 5.0); // 1/5th sec
      marker_array.markers[i].ns = "marker";
      marker_array.markers[i].id = i;

      marker_array.markers[i].type = visualization_msgs::Marker::CYLINDER;
      marker_array.markers[i].action = visualization_msgs::Marker::ADD;

      marker_array.markers[i].pose.position.x = map.at(i).x;
      marker_array.markers[i].pose.position.y = map.at(i).y;
      marker_array.markers[i].pose.position.z = 0.14;

      marker_array.markers[i].pose.orientation.x = 0.0;
      marker_array.markers[i].pose.orientation.y = 0.0;
      marker_array.markers[i].pose.orientation.z = 0.0;
      marker_array.markers[i].pose.orientation.w = 1.0;

      marker_array.markers[i].scale.x = 2.0 * 0.05;
      marker_array.markers[i].scale.y = 2.0 * 0.05;
      marker_array.markers[i].scale.z = 0.1;

      marker_array.markers[i].color.r = 0.0f;
      marker_array.markers[i].color.g = 0.0f;
      marker_array.markers[i].color.b = 1.0f;
      marker_array.markers[i].color.a = 1.0f;
    }
    marker_pub.publish(marker_array);


  }

  return 0;
}












// end file
