/// \file
/// \brief
///
/// \author Boston Cleek
/// \date 2/7/20
///
/// PUBLISHES:
/// cmd_vel (geometry_msgs/Twist): Commanded body twist
///
/// SERVICES:
/// start (start) - intiates the movement of the turtlebot and call set_pose service



#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>

#include <string>
#include <iostream>
#include <exception>

#include "nuturtle_robot/start.h"
#include "rigid2d/set_pose.h"
#include "rigid2d/rigid2d.hpp"


// global variable
static std::string direction;                          // direction of rotation
static bool rotation_srv_flag;                         // start srv flag
static int ctr;
static double rot;                                     // rotation speed
static double lin;                                     // translation speed

static bool doing_circles;                             // rotating
static bool doing_trans;                               // translating

static bool twenty_rotations;                          // 20 circles
static bool ten_translations;                          // 10 trans steps

static int total_rotations;                            // total number of rotations
static int total_trans;                                // total number of translation step




/// \brief service sets the rotation direction of the robot
/// \param req - service request direction
/// \param res - service direction result
/// \param set_pose_client - set pose client
/// \param set_fake_pose_client - set pose fake client
bool setDirectionService(nuturtle_robot::start::Request &req,
                         nuturtle_robot::start::Response &res,
                         ros::ServiceClient &set_pose_client,
                         ros::ServiceClient &set_fake_pose_client)
{

  // issue request and set response
  direction = req.direction;
  res.direction_set = true;

  // set flags
  rotation_srv_flag = true;
  twenty_rotations = false;
  ten_translations = false;


  // reset counters
  ctr = 0;
  total_rotations = 0;
  total_trans = 0;


  if (direction == "clockwise" or  direction == "counter-clockwise")
  {
   doing_circles = true;
  }

  if (direction == "forward" or  direction == "backward")
  {
   doing_trans = true;
  }

  // reset pose
  // reset pose to (0,0,0)
  if (ros::service::exists("set_pose", true) and ros::service::exists("fake/set_pose", true))
  {
    rigid2d::set_pose pose_srv;
    pose_srv.request.theta = 0.0;
    pose_srv.request.x = 0.0;
    pose_srv.request.y = 0.0;

    set_pose_client.call(pose_srv);
    set_fake_pose_client.call(pose_srv);
  }


  ROS_INFO("start service activated");



  return true;
}



/// \breif Publishes a twist using a time
/// \param event - provides timing information
/// \param cmd_pub - ros publisher
void cmdCallback(const ros::TimerEvent&, ros::Publisher &cmd_pub)
{
  geometry_msgs::Twist cmd;
  cmd.linear.x = lin;
  cmd.linear.y = 0;
  cmd.linear.z = 0;
  cmd.angular.x = 0;
  cmd.angular.y = 0;
  cmd.angular.z = rot;

  cmd_pub.publish(cmd);

  // make sure service has been called before counting
  if (rotation_srv_flag)
  {
    ctr++;
  }
}





int main(int argc, char** argv)
{

  ros::init(argc, argv, "rotation");
  ros::NodeHandle node_handle;

  ros::ServiceClient set_pose_client = node_handle.serviceClient<rigid2d::set_pose>("set_pose");
  ros::ServiceClient set_fake_pose_client = node_handle.serviceClient<rigid2d::set_pose>("fake/set_pose");


  ros::Publisher cmd_pub = node_handle.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::ServiceServer dir_server = node_handle.advertiseService<nuturtle_robot::start::Request,
                                    nuturtle_robot::start::Response>("start",
                                    std::bind(&setDirectionService, std::placeholders::_1,
                                              std::placeholders::_2, set_pose_client, set_fake_pose_client));


  double max_rot = 0.0, frac_vel = 0.0, max_trans = 0.0;

  node_handle.getParam("/max_rot", max_rot);
  node_handle.getParam("/max_trans", max_trans);
  node_handle.getParam("rotation/frac_vel", frac_vel);


  ROS_INFO("max_rot %f\n", max_rot);
  ROS_INFO("max_trans %f\n", max_trans);
  ROS_INFO("frac_vel %f\n", frac_vel);


  ROS_INFO("Successfully launched rotation node.");


  // timer frequency
  double frequency = 110;

  ros::Timer timer = node_handle.createTimer(ros::Duration(1 / frequency),
                       std::bind(&cmdCallback, std::placeholders::_1, cmd_pub));


  rotation_srv_flag = false;
  doing_circles = false;
  doing_trans = false;


  // rotation
  /////////////////////////////////////////////////////////////////////////////
  // when 20 rotations have been achieved
  twenty_rotations = false;

  // one circle
  bool circle_achieved = false;

  // number of timer calls for 1 revolution
  const auto one_circle = static_cast<int> (2.0*rigid2d::PI / (max_rot * frac_vel * (1.0 / frequency)));

  // counts to pause for 1/20th of a circle
  const auto one_twenty_circle = static_cast<int> ((1.0 / 20.0) * (2.0*rigid2d::PI / (max_rot * frac_vel * (1.0 / frequency))));

  /////////////////////////////////////////////////////////////////////////////


  // translation
  /////////////////////////////////////////////////////////////////////////////
  // when 10 trans steps of 0.2m done
  ten_translations = false;

    // one translation step achieved
  bool trans_achieved = false;

  // forward or backward for 0.2 m
  const auto one_trans =static_cast<int> (0.2 / (max_trans * frac_vel * (1.0 / frequency)));

  // pause for 1/10 of the time to go 0.2 m
  const auto one_ten_trans = static_cast<int> ((1.0 / 10.0) * (0.2 / (max_trans * frac_vel * (1.0 / frequency))));
  /////////////////////////////////////////////////////////////////////////////


  ROS_INFO("Timer calls per circle %d\n", one_circle);
  ROS_INFO("Timer calls per 1/20 circle %d\n", one_twenty_circle);


  ROS_INFO("Timer calls per translate %d\n", one_trans);
  ROS_INFO("Timer calls per 1/10 translate %d\n", one_ten_trans);

  while(node_handle.ok())
  {
    ros::spinOnce();


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

      else if (direction == "forward")
      {
        lin = (max_trans * frac_vel);
      }

      else if (direction == "backward")
      {
        lin = -(max_trans * frac_vel);
      }

      else
      {
        throw std::invalid_argument("Not a valid direction");
      }
    }

    // circles
    ///////////////////////////////////////////////////////////////////////////
    if (doing_circles)
    {

      // full rotation
      if (ctr == one_circle)
      {
        ctr = 0;
        circle_achieved = true;
        total_rotations++;
      }

      // pause for 1/20 rotation
      if (ctr <= one_twenty_circle and circle_achieved)
      {
        // pausing
        if (ctr != one_twenty_circle)
        {
          rot = 0.0;
        }

        // stop pausing
        else
        {
          ctr = 0;
          circle_achieved = false;
        }
      }


      // 20 rotations
      if (total_rotations == 20)
      {
        twenty_rotations = true;
      }


      if (twenty_rotations)
      {
        rot = 0.0;
      }
    }
    ///////////////////////////////////////////////////////////////////////////


    // translation
    ///////////////////////////////////////////////////////////////////////////
    if (doing_trans)
    {

      // full rotation
      if (ctr == one_trans)
      {
        ctr = 0;
        trans_achieved = true;
        total_trans++;
      }

      // pause for 1/10 the translation distance
      if (ctr <= one_ten_trans and trans_achieved)
      {
        // pausing
        if (ctr != one_ten_trans)
        {
          lin = 0.0;
        }

        // stop pausing
        else
        {
          ctr = 0;
          trans_achieved = false;
        }
      }


      // 10 translation
      if (total_trans == 10)
      {
        ten_translations = true;
      }


      if (ten_translations)
      {
        lin = 0.0;
      }

    }


    // ROS_INFO("counter %d", ctr);
    // ROS_INFO("rot %f", rot);


  }


  return 0;
}











// end file
