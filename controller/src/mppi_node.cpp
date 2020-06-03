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
#include "controller/rk4.hpp"



int main(int argc, char** argv)
{
  ros::init(argc, argv, "mppi");
  ros::NodeHandle nh("~");
  ros::NodeHandle node_handle;

  const auto dt = 0.1;
  controller::RK4 rk4(dt);




  return 0;
}



















// end file
