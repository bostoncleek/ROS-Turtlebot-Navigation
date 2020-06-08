/// \file
/// \brief Composes body twist using model predictive path integral control
///
/// \author Boston Cleek
/// \date 6/3/20
///
/// PUBLISHES:
///
///
/// SUBSCRIBES:
///
/// SEERVICES:
///
///

#include <ros/ros.h>
#include <ros/console.h>

#include <iostream>
#include <functional>
#include <cmath>
#include <eigen3/Eigen/Dense>

#include <rigid2d/rigid2d.hpp>
#include <rigid2d/diff_drive.hpp>

#include "controller/rk4.hpp"
#include "controller/mppi.hpp"


using namespace rigid2d;




int main(int argc, char** argv)
{
  ros::init(argc, argv, "mppi_waypoints");
  ros::NodeHandle nh("~");
  ros::NodeHandle node_handle;

  double wheel_radius = 0.033;
  double wheel_base = 0.16;

  const auto dt = 0.1;
  const auto tf = 1.0;
  controller::RK4 rk4(dt);

  Eigen::VectorXd x0(3);
  x0 << 0.0, 0.0, 0.0;


  const auto N = static_cast<int>(tf/dt);
  Eigen::MatrixXd u(2,N);
  for(int i = 0; i < N; i++)
  {
    u(0,i) = 6.0;
    u(1,i) = 3.0;
  }


  controller::CartModel cart_model(wheel_radius, wheel_base);


  std::function<void(const Eigen::Ref<Eigen::VectorXd>,
                     const Eigen:: Ref<Eigen::VectorXd>,
                      Eigen::Ref<Eigen::VectorXd>)> func_cntrl = std::bind(&controller::CartModel::kinematicCart,
                                                                  &cart_model,
                                                                  std::placeholders::_1,
                                                                  std::placeholders::_2,
                                                                  std::placeholders::_3);


  rk4.registerODE(func_cntrl);
  Eigen::MatrixXd sol = rk4.solve(x0, u, tf);
  std::cout << sol << std::endl;





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
