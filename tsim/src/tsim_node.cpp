/// \file   tsim_node.cpp
/// \brief  Describes a recatngular trajectory for a turtle
///
/// \author Boston Cleek
/// \date 1/9/20
///
/// PUBLISHES:
///     cmd_vel (Twist): linear and angular velocity controls
///     pose_error (PoseError): error in turtles pose
/// SUBSCRIBES:
///     turtlesim/Pose (Pose): pose of turtle
/// SERVICES:
///     traj_reset (Empty): resets turtle back to start



#include <ros/ros.h>
#include <ros/console.h>
#include <turtlesim/Pose.h>
#include <turtlesim/SetPen.h>
#include <turtlesim/TeleportAbsolute.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include "tsim/PoseError.h"

#include <string>
#include <vector>
#include <math.h>

using std::cout;
using std::endl;
using std::string;
using std::vector;


/// \brief The Control class provides the control loop that governs the
/// behavior of the turtle following a rectangular trajectory

class Control
{
public:

  /// \brief class constructor
  /// \param node_handle - the NodeHandle
  Control(ros::NodeHandle &node_handle);

  /// \brief destructor
  ~Control();

  /// \brief bang-bang control loop
  /// \returns void
  void bangBang();

  /// \brief feedforward control loop
  /// \returns void
  void FeedForward();


private:

  /// \brief tests if topics subscribed to are read
  /// \param subscriber_topic - topic subscribed to
  /// \returns bool
  bool readParameters(string subscriber_topic);

  /// \brief resets turtle to
  /// \param request - Empty request
  /// \param response - Empty response
  /// \returns bool
  bool trajCallBack(std_srvs::Empty::Request&, std_srvs::Empty::Response& );

  /// \brief updates the pose
  /// \param pose_msg - constant pointer to turtle Pose
  /// \returns void
  void poseCallBack(const turtlesim::Pose::ConstPtr& pose_msg);


  int x_;               // x - lower left corner of rectangle
  int y_;               // y - lower left corner of rectangle
  int width_;           // width of rectangle
  int height_;          // height of rectangle
  int trans_vel_;       // linear velocity
  int rot_vel_;         // angular velocity
  int frequency_;       // publishing rate of controls
  int ctr_;             // state for which corner the turtle is at
  int iter_;            // counter for determining time for states

  float h_tol_;         // heading tolerance of turtle
  float p_tol_;         // poistion tolerance of turtle
  bool init_position_;  // turtle starts from the beginning


  // error in pose
  tsim::PoseError error_msg_;

  // pose of turtle
  turtlesim::Pose pose_;

  // handle to node
  ros::NodeHandle &node_handle_;

  // subscriber to pose topic
  ros::Subscriber pose_sub_;

  // publisher for velocity controls
  ros::Publisher vel_pub_;

  // publisher for error in pose
  ros::Publisher error_pub_;

  // service for starting at the beginning of rectangle
  ros::ServiceServer traj_service_;

  // set turtle's pen turtlesim service
  ros::ServiceClient pen_client_;

  // teleport turtle turtlesim service
  ros::ServiceClient tele_client_;

  // teleport service object
  turtlesim::TeleportAbsolute tele_srv_;

  // set pen service object
  turtlesim::SetPen pen_srv_;
};


Control::Control(ros::NodeHandle &node_handle): node_handle_(node_handle)
{
  string pose_topic = "/turtle1/pose";
  h_tol_ = 0.175;
  p_tol_ = 0.1;
  init_position_ = false;

  if(!this->Control::readParameters("/turtle1/pose"))
  {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }

  // subscribe to pose of turtle
  pose_sub_ = node_handle_.subscribe(pose_topic, 1, &Control::poseCallBack, this);

  // publish velocity
  vel_pub_ = node_handle_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  // publish pose error
  error_pub_ = node_handle_.advertise<tsim::PoseError>("pose_error", 1);

  // reset trajectory service
  traj_service_ = node_handle_.advertiseService("traj_reset", &Control::trajCallBack, this);

  // teleport client
  tele_client_ = node_handle_.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");

  // pen client
  pen_client_ = node_handle_.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");

  // read in parameters from server
  node_handle.getParam("/x", x_);
  node_handle.getParam("/y", y_);
  node_handle.getParam("/width", width_);
  node_handle.getParam("/height", height_);
  node_handle.getParam("/trans_vel", trans_vel_);
  node_handle.getParam("/rot_vel", rot_vel_);
  node_handle.getParam("/frequency", frequency_);

  ROS_INFO("x: %d", x_);
  ROS_INFO("y: %d", y_);
  ROS_INFO("width: %d", width_);
  ROS_INFO("height: %d", height_);
  ROS_INFO("trans_vel: %d", trans_vel_);
  ROS_INFO("rot_vel: %d", rot_vel_);
  ROS_INFO("frequency: %d", frequency_);

  // wait for servies to be availible
  ros::service::waitForService("/turtle1/set_pen", -1);
  ros::service::waitForService("/turtle1/teleport_absolute", -1);


  // place turtle at lower left of rectangle
  // turn off pen
  pen_srv_.request.r = 0;
  pen_srv_.request.g = 0;
  pen_srv_.request.b = 0;
  pen_srv_.request.width = 0;
  pen_srv_.request.off = 1;

  pen_client_.call(pen_srv_);

  // teleport
  tele_srv_.request.x = x_;
  tele_srv_.request.y = y_;
  tele_srv_.request.theta = 0;

  tele_client_.call(tele_srv_);

  // turn pen back on
  pen_srv_.request.r = 255;
  pen_srv_.request.g = 255;
  pen_srv_.request.b = 255;
  pen_srv_.request.width = 2;
  pen_srv_.request.off = 0;

  pen_client_.call(pen_srv_);


  ROS_INFO("Successfully launched node.");
}


Control::~Control()
{
}


void Control::bangBang()
{

  ros::Rate loop_rate(frequency_);

  geometry_msgs::Twist twist_msg;
  tsim::PoseError error_msg;


  // states
  int waypoints[4][2] = {{x_, y_}, {x_ + width_, y_},
                         {x_ + width_, y_ + height_}, {x_, y_ + height_}};

  ctr_ = 1;
  while(ros::ok())
  {
    // current goal
    int x_pt = waypoints[ctr_][0];
    int y_pt = waypoints[ctr_][1];

    // bearing towards goal
    float b = atan2(y_pt - pose_.y, x_pt - pose_.x);

    // heading error
    float h_err = b - pose_.theta;

    // abs error in pose
    error_msg.x_error = abs(pose_.x - x_pt);
    error_msg.y_error = abs(pose_.y - y_pt);
    error_msg.theta_error = abs(h_err);


    // heading within tol and move toward goal
    if (abs(h_err) < h_tol_)
    {
      twist_msg.linear.x = trans_vel_;
      twist_msg.linear.y = 0;
      twist_msg.linear.z = 0;
      twist_msg.angular.x = 0;
      twist_msg.angular.y = 0;
      twist_msg.angular.z = 0;
    }

    // turn
    else
    {
      twist_msg.linear.x = 0;
      twist_msg.linear.y = 0;
      twist_msg.linear.z = 0;
      twist_msg.angular.x = 0;
      twist_msg.angular.y = 0;

      // wrap error to 0 -> 2pi
      if (h_err < 0)
        h_err += 2*M_PI;

      if (h_err <= M_PI and h_err >= 0 )
        twist_msg.angular.z = rot_vel_;
      else
        twist_msg.angular.z = -rot_vel_;
    }

    // check if goal reached
    float d = sqrt(pow(x_pt - pose_.x, 2) + pow(y_pt - pose_.y, 2));
    if (d < p_tol_)
    {
      ROS_INFO("Goal reached: %d", ctr_);
      ctr_++;

      // start back at first goal
      if (ctr_ > 3)
        ctr_ = 0;
    }


    // wait for turtle to be in correct starting position
    // publish new control
    if (!init_position_ and pose_.x == x_ and pose_.y == y_ and pose_.theta == 0)
    {
      ROS_INFO("At Starting position");
      init_position_ = true;
      // wait for velocities to publish to 0
      // else turtle may keep moving once teleported
      ros::Duration(0.5).sleep();


    }

    else if (init_position_)
      vel_pub_.publish(twist_msg);

    // publish the error in pose
    error_pub_.publish(error_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
}


void Control::FeedForward()
{

  ros::Rate loop_rate(frequency_);

  geometry_msgs::Twist twist_msg;
  tsim::PoseError error_msg;

  // time to execute states
  float horiz_time = (float)(width_) / (float)(trans_vel_);
  float vert_time = (float)(height_) / (float)(trans_vel_);
  float turn_time = (M_PI/2) / (float)(rot_vel_);
  float curr_time = 0;

  // states
  string states[3] = {"Horiz", "Turn", "Vert"};
  string st = "Horiz";
  string prev_st = "Turn";

  // pose estimate
  float x = x_;
  float y = y_;
  float theta = 0;


  iter_ = 0;
  while(ros::ok())
  {


    // set controls based on state
    if (st == "Horiz" and curr_time <= horiz_time)
    {
      twist_msg.linear.x = trans_vel_;
      twist_msg.linear.y = 0;
      twist_msg.linear.z = 0;
      twist_msg.angular.x = 0;
      twist_msg.angular.y = 0;
      twist_msg.angular.z = 0;
      //ROS_INFO("Horiz");
    }

    else if (st == "Turn" and curr_time <= turn_time)
    {
      twist_msg.linear.x = 0;
      twist_msg.linear.y = 0;
      twist_msg.linear.z = 0;
      twist_msg.angular.x = 0;
      twist_msg.angular.y = 0;
      twist_msg.angular.z = rot_vel_;
      //ROS_INFO("Turn");
    }

    else if (st == "Vert" and curr_time <= vert_time)
    {
      twist_msg.linear.x = trans_vel_;
      twist_msg.linear.y = 0;
      twist_msg.linear.z = 0;
      twist_msg.angular.x = 0;
      twist_msg.angular.y = 0;
      twist_msg.angular.z = 0;
      //ROS_INFO("Vert");
    }


    // state transitions
    if (st == "Horiz" and prev_st == "Turn" and curr_time >= horiz_time)
    {
      st = "Turn";
      prev_st = "Horiz";
      iter_ = 0;
    }

    else if (st == "Turn" and prev_st == "Horiz" and curr_time >= turn_time)
    {
      st = "Vert";
      prev_st = "Turn";
      iter_ = 0;
    }


    else if (st == "Vert" and prev_st == "Turn" and curr_time >= vert_time)
    {
      st = "Turn";
      prev_st = "Vert";
      iter_ = 0;
    }

    else if (st == "Turn" and prev_st == "Vert" and curr_time >= turn_time)
    {
      st = "Horiz";
      prev_st = "Turn";
      iter_ = 0;
    }

    // update pose estimate
    x += twist_msg.linear.x * std::cos(theta) * 1.0 / (float) frequency_;
    y += twist_msg.linear.x * std::sin(theta) * 1.0 / (float) frequency_;
    theta += twist_msg.angular.z * 1.0 / (float) frequency_;

    // wrap heading -PI -> PI
    theta = std::fmod(theta + M_PI, 2*M_PI);
    if (theta < 0)
      theta += 2*M_PI;

    theta -= M_PI;

    // errors in pose
    error_msg.x_error = std::abs(pose_.x - x);
    error_msg.y_error = std::abs(pose_.y - y);
    error_msg.theta_error = std::abs(std::abs(pose_.theta) - std::abs(theta));


    // wait for turtle to be in correct starting position
    if (!init_position_ and pose_.x == x_ and pose_.y == y_ and pose_.theta == 0)
    {
      ROS_INFO("At Starting position");
      init_position_ = true;
      iter_ = 0;
      st = "Horiz";
      prev_st = "Turn";

      x = x_;
      y = y_;
      theta = 0;

    }

    // publish new control
    else if (init_position_)
      vel_pub_.publish(twist_msg);

    // publish the error in pose
    error_pub_.publish(error_msg);


    iter_++;
    curr_time = (float)(iter_) / (float)(frequency_);

    ros::spinOnce();
    loop_rate.sleep();

  }
}


bool Control::readParameters(string subscriber_topic)
{
  return (!node_handle_.getParam("subscriber_topic", subscriber_topic)) ? true : false;
}


bool Control::trajCallBack(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{

  // publish no controls to make sure turtle does not
  // start moving when it spawns
  geometry_msgs::Twist twist_msg;
  twist_msg.linear.x = 0;
  twist_msg.linear.y = 0;
  twist_msg.linear.z = 0;
  twist_msg.angular.x = 0;
  twist_msg.angular.y = 0;
  twist_msg.angular.y = 0;

  vel_pub_.publish(twist_msg);

  // lift pen
  pen_srv_.request.r = 0;
  pen_srv_.request.g = 0;
  pen_srv_.request.b = 0;
  pen_srv_.request.width = 0;
  pen_srv_.request.off = 1;

  // turn off pen
  pen_client_.call(pen_srv_);

  // teleport
  tele_client_.call(tele_srv_);

  // turn pen back on
  pen_srv_.request.r = 255;
  pen_srv_.request.g = 255;
  pen_srv_.request.b = 255;
  pen_srv_.request.width = 2;
  pen_srv_.request.off = 0;

  pen_client_.call(pen_srv_);

  // set the state back to the forst goal
  ctr_ = 1;
  init_position_ = false;

  return true;

}


void Control::poseCallBack(const turtlesim::Pose::ConstPtr& pose_msg)
{
  pose_ = *pose_msg;
}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtle_rect");
  ros::NodeHandle node_handle;//("~");

  Control control(node_handle);
  //control.bangBang();
  control.FeedForward();

  ros::spin();
  return 0;
}














// end file
