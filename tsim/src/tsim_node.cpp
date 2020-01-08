//  \file







// FILL THS OUT !!!!!!!!!!!!!





#include <ros/ros.h>
#include <ros/console.h>
#include <turtlesim/Pose.h>
#include <turtlesim/SetPen.h>
#include <turtlesim/TeleportAbsolute.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
//#include <std_msgs/String.h>
//#include <geometry_msgs/Vector3.h>
#include "tsim/PoseError.h"

#include <string>
#include <vector>
#include <math.h>

using std::cout;
using std::endl;
using std::string;
using std::vector;


class Control
{
public:
  Control(ros::NodeHandle &node_handle);
  ~Control();
  void controlLoop();

private:
  bool readParameters(string subscriber_topic);
  bool trajCallBack(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  void poseCallBack(const turtlesim::Pose::ConstPtr& pose_msg);

  int x_, y_, width_, height_, trans_vel_, rot_vel_, frequency_;
  int ctr_;
  float h_tol_, p_tol_;
  bool init_position_;

  tsim::PoseError error_msg_;

  turtlesim::Pose pose_;

  ros::NodeHandle &node_handle_;
  ros::Subscriber pose_sub_;
  ros::Publisher vel_pub_;
  ros::Publisher error_pub_;

  ros::ServiceServer traj_service_;

  ros::ServiceClient pen_client_;
  ros::ServiceClient tele_client_;

  turtlesim::TeleportAbsolute tele_srv_;
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
  pose_sub_ = node_handle_.subscribe(pose_topic, 10, &Control::poseCallBack, this);

  // publish velocity
  vel_pub_ = node_handle_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  // publish pose error
  error_pub_ = node_handle_.advertise<tsim::PoseError>("pose_error", 10);

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

  //ros::Duration(0.5).sleep();

  ros::service::waitForService("/turtle1/set_pen", -1);
  ros::service::waitForService("/turtle1/teleport_absolute", -1);


  // place turtle at lower left of rectangle
  // if(ros::service::exists("/turtle1/set_pen", true) and ros::service::exists("/turtle1/teleport_absolute", true))
  // {
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
  // }


  ROS_INFO("Successfully launched node.");
}


Control::~Control()
{
}


void Control::controlLoop()
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


bool Control::readParameters(string subscriber_topic)
{
  return (!node_handle_.getParam("subscriber_topic", subscriber_topic)) ? true : false;
}


bool Control::trajCallBack(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{

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
  control.controlLoop();

  ros::spin();
  return 0;
}














// end file
