/// \file
/// \brief Composes body twist to move turtle in pentagon
///
/// \author Boston Cleek
/// \date 2/14/20
///
/// PUBLISHES:
///   cmd_vel (Twist): linear and angular velocity controls
///   vizualize_waypoints (visualization_msgs::MarkerArray): Marker array representing waypoint
/// SUBSCRIBES:
///   odom (nav_msgs/Odometry): The pose of the turtlebot from odometer node
/// SEERVICES:
///   start (nuturtle_robot/Start): resets pose and starts waypoints following
///   stop (Empty): stops turtlebot movement



#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


#include <string>
#include <vector>
#include <iostream>

#include <rigid2d/rigid2d.hpp>
#include <rigid2d/diff_drive.hpp>
#include <rigid2d/waypoints.hpp>
#include "rigid2d/set_pose.h"
#include "nuturtle_robot/start.h"



// global variables
rigid2d::Pose pose;

static bool odom_msg;
static bool start_call;
static bool stop_call;


/// \brief Publishes array of markers as waypoints
/// \param pts - waypoints
void placeMarkers(ros::Publisher &marker_pub,
                  const std::vector<rigid2d::Vector2D> &pts)
{
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.resize(pts.size());

  for(unsigned int i = 0; i < pts.size(); i++)
  {
    marker_array.markers[i].header.frame_id = "odom";
    marker_array.markers[i].header.stamp = ros::Time::now();
    marker_array.markers[i].lifetime = ros::Duration();
    marker_array.markers[i].ns = "marker";
    marker_array.markers[i].id = i;

    marker_array.markers[i].type = visualization_msgs::Marker::CYLINDER;
    marker_array.markers[i].action = visualization_msgs::Marker::ADD;

    marker_array.markers[i].pose.position.x = pts.at(i).x;
    marker_array.markers[i].pose.position.y = pts.at(i).y;
    marker_array.markers[i].pose.position.z = 0.0;

    marker_array.markers[i].pose.orientation.x = 0.0;
    marker_array.markers[i].pose.orientation.y = 0.0;
    marker_array.markers[i].pose.orientation.z = 0.0;
    marker_array.markers[i].pose.orientation.w = 1.0;

    marker_array.markers[i].scale.x = 0.1;
    marker_array.markers[i].scale.y = 0.1;
    marker_array.markers[i].scale.z = 0.1;

    marker_array.markers[i].color.r = 0.0f;
    marker_array.markers[i].color.g = 1.0f;
    marker_array.markers[i].color.b = 0.0f;
    marker_array.markers[i].color.a = 1.0f;
  }
  marker_pub.publish(marker_array);
}


/// \brief service sets the rotation direction of the robot
/// \param req - service request direction
/// \param res - service direction result
/// \param set_pose_client - set pose client
/// \param marker_pub - marker publisher
/// \param pts - waypoints
bool setStartService(nuturtle_robot::start::Request &,
                     nuturtle_robot::start::Response &res,
                     ros::ServiceClient &set_pose_client,
                     ros::Publisher &marker_pub,
                     const std::vector<rigid2d::Vector2D> &pts)
{
  res.direction_set = true;

  if (ros::service::exists("set_pose", true))
  {
    rigid2d::set_pose pose_srv;
    pose_srv.request.theta = 0.0;
    pose_srv.request.x = pts.at(0).x;
    pose_srv.request.y = pts.at(0).y;

    set_pose_client.call(pose_srv);

    placeMarkers(marker_pub, pts);

  }

  start_call = true;
  stop_call = false;
  return true;
}


/// \brief Stops turtlebot movement
/// \param Request - empty request
/// \param Response - empty response
bool setStopService(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  stop_call = true;
  start_call = false;

  return true;
}


/// \brief Updates pose of robot
/// \param msg - odometry feedback
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  pose.x = msg->pose.pose.position.x;
  pose.y = msg->pose.pose.position.y;

  auto roll = 0.0, pitch = 0.0, yaw = 0.0;
  tf2::Quaternion quat(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf2::Matrix3x3 mat(quat);
  mat.getRPY(roll, pitch, yaw);

  pose.theta = yaw;

  odom_msg = true;
}





int main(int argc, char** argv)
{
  ros::init(argc, argv, "real_waypoint");
  ros::NodeHandle nh("~");
  ros::NodeHandle node_handle;

  ros::Subscriber joint_sub = node_handle.subscribe("odom", 1, odomCallback);

  ros::Publisher vel_pub = node_handle.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Publisher marker_pub = node_handle.advertise<visualization_msgs::MarkerArray>("vizualize_waypoints", 100, true);

  ros::ServiceClient set_pose_client = node_handle.serviceClient<rigid2d::set_pose>("set_pose");

  // velocity limits
  double max_rot = 0.0, frac_vel = 0.0, max_trans = 0.0;

  // waypoints
  std::vector<float> waypoint_x;
  std::vector<float> waypoint_y;

  // load parameters
  node_handle.getParam("/max_rot", max_rot);
  node_handle.getParam("/max_trans", max_trans);
  nh.getParam("frac_vel", frac_vel);

  nh.getParam("x_component", waypoint_x);
  nh.getParam("y_component", waypoint_y);

  // load waypoint
  std::vector<rigid2d::Vector2D> pts;
  // ROS_INFO("Waypoints vector size %zd", waypoint_x.size());
  for(unsigned int i = 0; i < waypoint_x.size(); i++)
  {
    rigid2d::Vector2D v;
    v.x = waypoint_x.at(i);
    v.y = waypoint_y.at(i);
    pts.push_back(v);
  }

  // starts robot at 1st waypoint
  ros::ServiceServer start_server = node_handle.advertiseService<nuturtle_robot::start::Request,
                                    nuturtle_robot::start::Response>("start",
                                    std::bind(&setStartService, std::placeholders::_1,
                                              std::placeholders::_2, set_pose_client,
                                              marker_pub, pts));


  ros::ServiceServer stop_server = node_handle.advertiseService("stop", setStopService);


  ROS_INFO("Successfully launched real_waypoint node.");


  odom_msg = false;
  start_call = false;
  stop_call = false;

  // twist control
  rigid2d::Twist2D cmd;
  geometry_msgs::Twist twist_msg;
  twist_msg.linear.x = 0.0;
  twist_msg.linear.y = 0.0;
  twist_msg.linear.z = 0.0;
  twist_msg.angular.x = 0.0;
  twist_msg.angular.y = 0.0;
  twist_msg.angular.z = 0.0;

  // waypoints controller
  rigid2d::Waypoints waypts(pts, max_rot, max_trans);

  // set gains
  // K rot: 1
  waypts.setGain(1.0);

  // loop rate
  int frequency = 60;
  ros::Rate loop_rate(frequency);


  while(node_handle.ok())
  {
    ros::spinOnce();


    // update constrols
    if (odom_msg and start_call)
    {
      cmd = waypts.nextWaypointClosedLoop(pose);
      twist_msg.linear.x = cmd.vx;
      twist_msg.angular.z = cmd.w;
    }


    // publish twist and markers if robot started
    if (!stop_call)
    {
      vel_pub.publish(twist_msg);
    }

    loop_rate.sleep();
  }



  return 0;
}












// end file
