/// \file
/// \brief Composes body twist using model predictive path integral control
///
/// \author Boston Cleek
/// \date 6/3/20
///
/// PUBLISHES:
///   cmd_vel (geometry_msgs/Twist): twist in body frame
/// SUBSCRIBES:
///   odom (nav_msgs/Odometry): Pose of robot in odom frame
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

#include <iostream>
#include <functional>
#include <cmath>
#include <vector>
#include <eigen3/Eigen/Dense>

#include <rigid2d/rigid2d.hpp>
#include <rigid2d/diff_drive.hpp>
#include <controller/rk4.hpp>
#include <controller/mppi.hpp>


static rigid2d::Pose pose;
static bool odom_msg;


/// \brief Updates pose of robot
/// \param msg - odometry feedback
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


int main(int argc, char** argv)
{
  ros::init(argc, argv, "mppi_waypoints");
  ros::NodeHandle nh("~");
  ros::NodeHandle node_handle;

  ros::Subscriber odom_sub = node_handle.subscribe("odom", 1, odomCallBack);
  ros::Publisher cmd_pub = node_handle.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  // set globals
  odom_msg = false;

  // turtlebot parameters
  auto wheel_radius = 0.0;          // wheel radius
  auto wheel_base = 0.0;            // wheel base
  auto max_rot_motor = 0.0;         // max wheel rotational speed (rad/s)

  // mppi sampling and cost parameters
  auto ul_var = 0.0;                // sampling variance
  auto ur_var = 0.0;                // sampling variance
  auto horizon = 0.0;               // time horizon
  auto time_step = 0.0;             // dt
  auto rollouts = 0;                // number of rollouts

  std::vector<double> Q;            // penalize states
  std::vector<double> R;            // penalize controls
  std::vector<double> P1;           // penalize terminal states

  // waypoints
  std::vector<float> waypoint_x;     // waypoint x coordinates
  std::vector<float> waypoint_y;     // waypoint y coordinates



  node_handle.getParam("/wheel_radius", wheel_radius);
  node_handle.getParam("/wheel_base", wheel_base);
  node_handle.getParam("/max_rot_motor", max_rot_motor);


  nh.getParam("ul_var", ul_var);
  nh.getParam("ur_var", ur_var);
  nh.getParam("horizon", horizon);
  nh.getParam("time_step", time_step);
  nh.getParam("rollouts", rollouts);
  nh.getParam("Q", Q);
  nh.getParam("R", R);
  nh.getParam("P1", P1);
  nh.getParam("waypoint_x", waypoint_x);
  nh.getParam("waypoint_y", waypoint_y);


  ROS_INFO("Successfully launched mppi_waypoints node.");
  /////////////////////////////////////////////////////////////////////////////



  // while(node_handle.ok())
  // {
  //   ros::spinOnce();
  //
  //   if (odom_msg)
  //   {
  //     std::cout << "msg" << std::endl;
  //     odom_msg = false;
  //   }
  // }



  // double wheel_radius = 0.033;
  // double wheel_base = 0.16;
  //
  // const auto dt = 0.1;
  // const auto tf = 1.0;
  // controller::RK4 rk4(dt);
  //
  // Eigen::VectorXd x0(3);
  // x0 << 0.0, 0.0, 0.0;
  //
  //
  // const auto N = static_cast<int>(tf/dt);
  // Eigen::MatrixXd u(2,N);
  // for(int i = 0; i < N; i++)
  // {
  //   u(0,i) = 6.0;
  //   u(1,i) = 3.0;
  // }
  //
  //
  // controller::CartModel cart_model(wheel_radius, wheel_base);
  //
  //
  // std::function<void(const Eigen::Ref<Eigen::VectorXd>,
  //                    const Eigen:: Ref<Eigen::VectorXd>,
  //                     Eigen::Ref<Eigen::VectorXd>)> func_cntrl = std::bind(&controller::CartModel::kinematicCart,
  //                                                                 &cart_model,
  //                                                                 std::placeholders::_1,
  //                                                                 std::placeholders::_2,
  //                                                                 std::placeholders::_3);
  //
  //
  // rk4.registerODE(func_cntrl);
  // Eigen::MatrixXd sol = rk4.solve(x0, u, tf);
  // std::cout << sol << std::endl;





  // double wheel_radius = 0.033;
  // double wheel_base = 0.16;
  //
  // double x = 0.0;
  // double y = 1.0;
  // double theta = 0.5;
  //
  //
  // Pose pose;
  // pose.theta = theta;
  // pose.x = x;
  // pose.y = y;
  //
  // WheelVelocities vel;
  // vel.ul = 0.01;
  // vel.ur = 0.05;
  //
  //
  // // model
  // DiffDrive diff_drive(pose, wheel_base, wheel_radius);
  //
  // // Tsb
  // Vector2D v(x, y);
  // Transform2D Tsb(v, theta);
  //
  //
  // // T between q and b
  // Transform2D Tqb(theta);
  //
  //
  // // Vq
  // Twist2D Vq;
  // Vq.w = (wheel_radius/wheel_base) * (vel.ur - vel.ul);
  // Vq.vx = (wheel_radius/2.0) * (vel.ur + vel.ul) * std::cos(theta);
  // Vq.vy = (wheel_radius/2.0) * (vel.ur + vel.ul) * std::sin(theta);
  // std::cout << Vq << std::endl;
  //
  // // Vb from diff_drive model
  // Twist2D Vb = diff_drive.wheelsToTwist(vel);
  // std::cout << Vb << std::endl;
  //
  //
  // std::cout << Tqb(Vb) << std::endl;
  //
  // std::cout << diff_drive.twistToWheels(Tqb.inv()(Vq)).ul << std::endl;
  // std::cout << diff_drive.twistToWheels(Tqb.inv()(Vq)).ur << std::endl;

  return 0;
}



















// end file
