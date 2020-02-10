/// \file
/// \brief
///
/// \author Boston Cleek
/// \date 2/7/20
///
/// PUBLISHES:
///
///
/// SUBSCRIBES:
///



#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>

#include <string>
#include <iostream>
#include <exception>

#include "nuturtle_robot/start.h"
#include "rigid2d/set_pose.h"



// global variable
static std::string direction;                          // direction of rotation
static bool rotation_srv_flag;                         // start srv flag
static int ctr;
static double rot;                                       // rotation speed



/// \brief service sets the rotation direction of the robot
/// \param req - service request direction
/// \param res - service direction result
bool setDirectionService(nuturtle_robot::start::Request &req,
                         nuturtle_robot::start::Response &res,
                         ros::ServiceClient &set_pose_client)
{

  // issue request and set response
  direction = req.direction;
  res.direction_set = true;

  // set flag
  rotation_srv_flag = true;

  ROS_INFO("start service activated");


  // reset pose to (0,0,0)
  if (ros::service::exists("/set_pose", true))
  {
    rigid2d::set_pose pose_srv;
    pose_srv.request.theta = 0.0;
    pose_srv.request.x = 0.0;
    pose_srv.request.y = 0.0;

    set_pose_client.call(pose_srv);
  }


  return true;
}


/// \breif Publishes a twist using a time
/// \param event - provides timing information
void cmdCallback(const ros::TimerEvent& event, ros::Publisher &cmd_pub)
{
  geometry_msgs::Twist cmd;
  cmd.linear.x = 0;
  cmd.linear.y = 0;
  cmd.linear.z = 0;
  cmd.angular.x = 0;
  cmd.angular.y = 0;
  cmd.angular.z = rot;

  cmd_pub.publish(cmd);

  ctr++;
}




int main(int argc, char** argv)
{

  ros::init(argc, argv, "rotation");
  ros::NodeHandle node_handle;

  ros::ServiceClient set_pose_client = node_handle.serviceClient<rigid2d::set_pose>("/set_pose");


  ros::Publisher cmd_pub = node_handle.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::ServiceServer dir_server = node_handle.advertiseService<nuturtle_robot::start::Request,
                                    nuturtle_robot::start::Response>("/start",
                                    std::bind(&setDirectionService, std::placeholders::_1,
                                              std::placeholders::_2, set_pose_client));




  double max_rot = 0.0, frac_vel = 0.0;

  node_handle.getParam("/max_rot", max_rot);
  node_handle.getParam("/rotation/frac_vel", frac_vel);

  ROS_INFO("max_rot %f\n", max_rot);
  ROS_INFO("frac_vel %f\n", frac_vel);


  ROS_INFO("Successfully launched rotation node.");

  double frequency = 10;


  ros::Timer timer = node_handle.createTimer(ros::Duration(1 / frequency),
                       std::bind(&cmdCallback, std::placeholders::_1, cmd_pub));



  while(node_handle.ok())
  {
    ros::spinOnce();

    // ROS_INFO("counter %d", ctr);

    // full rotation
    if (ctr == 487)
    {
      ctr = 0;
    }

    // ROS_INFO("rot %f", rot);

    // start service as set direction
    if (rotation_srv_flag)
    {
      if (direction == "clockwise")
      {
        rot = max_rot * frac_vel;
      }

      else if (direction == "counter-clockwise")
      {
        rot = -(max_rot * frac_vel);
      }

      else
      {
        throw std::invalid_argument("Not a valid direction");
      }
    }


  }


  return 0;
}











// end file
