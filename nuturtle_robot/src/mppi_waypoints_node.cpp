/// \file
/// \brief Composes body twist using model predictive path integral control
///
/// \author Boston Cleek
/// \date 6/3/20
///
/// PUBLISHES:
///   cmd_vel (geometry_msgs/Twist): twist in body frame
///   odom_path (nav_msgs/Path): path executed by robot
/// SUBSCRIBES:
///   odom (nav_msgs/Odometry): Pose of robot in odom frame
/// SEERVICES:
///   start (nuturtle_robot/Start): resets pose and starts waypoints following
///   stop (Empty): stops turtlebot movement


// TODO : add filter to control signal



#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>
#include <functional>
#include <chrono>
#include <cmath>
#include <vector>
#include <eigen3/Eigen/Dense>

#include <rigid2d/rigid2d.hpp>
#include <rigid2d/utilities.hpp>
#include <rigid2d/diff_drive.hpp>
#include <controller/rk4.hpp>
#include <controller/mppi.hpp>
#include <rigid2d/set_pose.h>
#include "nuturtle_robot/start.h"


static rigid2d::Pose pose;                  // current pose of robot
static int wpt_id;                          // waypoint number
static bool odom_msg;                       // odometry message
static bool start_call;                     // call to start motion activated
static bool stop_call;                      // call to stop motion activated




/// \brief Updates pose of robot
/// \param msg - odometry feedback
void odomCallBack(const nav_msgs::Odometry::ConstPtr &msg);


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
                     const std::vector<std::vector<double>> &pts);


/// \brief Stops turtlebot movement
/// \param Request - empty request
/// \param Response - empty response
bool setStopService(std_srvs::Empty::Request&, std_srvs::Empty::Response&);


/// \brief Publishes array of markers as waypoints
/// \param pts - waypoints
void placeMarkers(ros::Publisher &marker_pub, const std::vector<std::vector<double>> &pts);




int main(int argc, char** argv)
{
  ros::init(argc, argv, "mppi_waypoints");
  ros::NodeHandle nh("~");
  ros::NodeHandle node_handle;

  ros::Subscriber odom_sub = node_handle.subscribe("odom", 1, odomCallBack);
  ros::Publisher cmd_pub = node_handle.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Publisher marker_pub = node_handle.advertise<visualization_msgs::MarkerArray>("vizualize_waypoints", 100, true);
  // ros::Publisher odom_path_pub = node_handle.advertise<nav_msgs::Path>("odom_path", 1);
  ros::ServiceClient set_pose_client = node_handle.serviceClient<rigid2d::set_pose>("set_pose");


  // set globals
  odom_msg = false;
  start_call = false;
  stop_call = false;

  // turtlebot parameters
  auto wheel_radius = 0.0;          // wheel radius
  auto wheel_base = 0.0;            // wheel base
  auto max_rot_motor = 0.0;         // max wheel rotational speed (rad/s)

  // mppi sampling and cost parameters
  auto lambda = 0.0;                // temperature parameter
  auto ul_var = 0.0;                // sampling variance
  auto ur_var = 0.0;                // sampling variance
  auto ul_init = 0.0;               // initial control left wheel
  auto ur_init = 0.0;               // initial control right wheel
  auto horizon = 0.0;               // time horizon
  auto time_step = 0.0;             // dt
  auto rollouts = 0;                // number of rollouts

  std::vector<double> Q;            // penalize states
  std::vector<double> R;            // penalize controls
  std::vector<double> P1;           // penalize terminal states

  // threshold to goal
  auto goal_thresh = 0.0;

  // frame of trajectory
  std::string frame_id;

  // waypoints
  std::vector<float> waypoint_x;         // waypoint x coordinates
  std::vector<float> waypoint_y;         // waypoint y coordinates
  std::vector<float> waypoint_theta;     // waypoint heading

  node_handle.getParam("/wheel_radius", wheel_radius);
  node_handle.getParam("/wheel_base", wheel_base);
  node_handle.getParam("/max_rot_motor", max_rot_motor);


  nh.getParam("lambda", lambda);
  nh.getParam("ul_var", ul_var);
  nh.getParam("ur_var", ur_var);
  nh.getParam("ul_init", ul_init);
  nh.getParam("ur_init", ur_init);
  nh.getParam("horizon", horizon);
  nh.getParam("time_step", time_step);
  nh.getParam("rollouts", rollouts);
  nh.getParam("Q", Q);
  nh.getParam("R", R);
  nh.getParam("P1", P1);
  nh.getParam("goal_thresh", goal_thresh);
  nh.getParam("odom_frame_id", frame_id);
  nh.getParam("x_component", waypoint_x);
  nh.getParam("y_component", waypoint_y);
  nh.getParam("theta_component", waypoint_theta);


  // load waypoint
  std::vector<std::vector<double>> pts;
  // ROS_INFO("Waypoints vector size %zd", waypoint_x.size());
  for(unsigned int i = 0; i < waypoint_x.size(); i++)
  {
    std::vector<double> p;
    p.push_back(waypoint_x.at(i));
    p.push_back(waypoint_y.at(i));
    p.push_back(waypoint_theta.at(i));
    pts.push_back(p);
  }

  // starts robot at 1st waypoint
  ros::ServiceServer start_server = node_handle.advertiseService<nuturtle_robot::start::Request,
                                    nuturtle_robot::start::Response>("start",
                                    std::bind(&setStartService, std::placeholders::_1,
                                              std::placeholders::_2, set_pose_client,
                                              marker_pub, pts));

  // stops robot
  ros::ServiceServer stop_server = node_handle.advertiseService("stop", setStopService);


  ROS_INFO("Successfully launched mppi_waypoints node.");
  /////////////////////////////////////////////////////////////////////////////

  controller::CartModel cart_model(wheel_radius, wheel_base);
  controller::LossFunc loss_func(Q, R, P1);
  controller::MPPI mppi(cart_model,
                        loss_func,
                        lambda,
                        max_rot_motor,
                        ul_var,
                        ur_var,
                        horizon,
                        time_step,
                        rollouts);


  mppi.setInitialControls(ul_init, ur_init);


  // WARNING: The pose (internal to diff_drive) will not be correct
  //          if the first waypoint is not at (0,0,0). But the internal
  //          pose is not used here.
  rigid2d::DiffDrive diff_drive(pose, wheel_base, wheel_radius);

  // drive to first waypoint
  wpt_id = 0;

  rigid2d::Pose wpt;
  wpt.x = waypoint_x.at(wpt_id);
  wpt.y = waypoint_y.at(wpt_id);
  wpt.theta = waypoint_theta.at(wpt_id);

  // set waypoint
  mppi.setWaypoint(wpt);

  // vizualize trajectory of robot
  // nav_msgs::Path odom_path;
  // odom_path.header.frame_id = frame_id;

  auto cnt = 0;
  geometry_msgs::Twist twist_msg;
  bool cycle_complete = false;

  while(node_handle.ok())
  {
    ros::spinOnce();

    // update constrols
    if (odom_msg and start_call)
    {
      // distance to goal
      const auto d2g = rigid2d::euclideanDistance(wpt.x, wpt.y, pose.x, pose.y);
      // std::cout << "Distance to goal: " << d2g << std::endl;
      if (d2g < goal_thresh)
      {
        ROS_INFO("Reached Waypoint %d", wpt_id);

        wpt_id++;
        cnt++;
        if (wpt_id % waypoint_x.size() == 0)
        {
          wpt_id = 0;
        }

        if (cnt == static_cast<int>(waypoint_x.size()+1))
        {
          cycle_complete = true;
          ROS_INFO("One cycle complete");
        }

        wpt.x = waypoint_x.at(wpt_id);
        wpt.y = waypoint_y.at(wpt_id);
        wpt.theta = waypoint_theta.at(wpt_id);

        mppi.setWaypoint(wpt);
      }

      // ////////////////////////////////////////////////////////////////////////////
      // // START TIMR
      // const auto start = std::chrono::high_resolution_clock::now();
      // ////////////////////////////////////////////////////////////////////////////

      rigid2d::WheelVelocities wheel_vel = mppi.newControls(pose);

      // ////////////////////////////////////////////////////////////////////////////
      // // STOP TIMER
      // const auto stop = std::chrono::high_resolution_clock::now();
      // const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
      // std::cout.precision(5);
      // std::cout << "Freq: " << (double)(1.0/(duration.count()/1000.0)) << " (Hz)" << std::endl;
      // //////////////////////////////////////////////////////////////////////////


      rigid2d::Twist2D cmd = diff_drive.wheelsToTwist(wheel_vel);

      twist_msg.linear.x = cmd.vx;
      twist_msg.angular.z = cmd.w;

      odom_msg = false;
    }

    // publish twist and markers if robot started
    if (!stop_call and !cycle_complete)
    {
      cmd_pub.publish(twist_msg);
    }


    // tf2::Quaternion q_or;
    // q_or.setRPY(0, 0, pose.theta);
    // geometry_msgs::Quaternion quat_or;
    // quat_or = tf2::toMsg(q_or);
    //
    // geometry_msgs::PoseStamped odom_pose;
    // odom_pose.header.stamp = ros::Time::now();
    // odom_pose.pose.position.x = pose.x;
    // odom_pose.pose.position.y = pose.y;
    // odom_pose.pose.orientation = quat_or;
    //
    // // odom_path.poses.emplace_back(odom_pose);
    //
    // odom_path_pub.publish(odom_path);
  }

  return 0;
}



void odomCallBack(const nav_msgs::Odometry::ConstPtr &msg)
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


bool setStartService(nuturtle_robot::start::Request &,
                     nuturtle_robot::start::Response &res,
                     ros::ServiceClient &set_pose_client,
                     ros::Publisher &marker_pub,
                     const std::vector<std::vector<double>> &pts)
{
  res.direction_set = true;

  if (ros::service::exists("set_pose", true))
  {
    rigid2d::set_pose pose_srv;

    pose_srv.request.x = pts.at(0).at(0);
    pose_srv.request.y = pts.at(0).at(1);
    pose_srv.request.theta = pts.at(0).at(2);
    // pose_srv.request.theta = rigid2d::PI/2.0;

    set_pose_client.call(pose_srv);

    // start driving to first waypoint
    // wpt_id = 1;

    // fill marker array
    placeMarkers(marker_pub, pts);

  }

  start_call = true;
  stop_call = false;
  return true;
}


bool setStopService(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  stop_call = true;
  start_call = false;

  return true;
}


void placeMarkers(ros::Publisher &marker_pub, const std::vector<std::vector<double>> &pts)
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

    marker_array.markers[i].pose.position.x = pts.at(i).at(0);
    marker_array.markers[i].pose.position.y = pts.at(i).at(1);
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


// end file
