#include <ros/ros.h>
#include <ros/console.h>
#include <turtlesim/Pose.h>
#include <turtlesim/SetPen.h>
#include <turtlesim/TeleportAbsolute.h>
#include <geometry_msgs/Twist.h>
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
  void poseCallBack(const turtlesim::Pose::ConstPtr& pose_msg);

  int x_, y_, width_, height_, trans_vel_, rot_vel_, frequency_;
  int tol_;


  turtlesim::Pose pose_;

  ros::NodeHandle &node_handle_;
  ros::Subscriber pose_sub_;
  ros::Publisher vel_pub_;
  ros::ServiceClient pen_client_;
  ros::ServiceClient tele_client_;

  turtlesim::TeleportAbsolute tele_srv_;
  turtlesim::SetPen pen_srv_;
};


Control::Control(ros::NodeHandle &node_handle): node_handle_(node_handle)
{

  string pose_topic = "/turtle1/pose";
  tol_ = 0.1;

  if(!this->Control::readParameters("/turtle1/pose"))
  {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }

  // subscribe to pose of turtle
  pose_sub_ = node_handle_.subscribe(pose_topic, 10, &Control::poseCallBack, this);

  // publish velocity
  vel_pub_ = node_handle_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);


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


  // teleport client
  ros::ServiceClient tele_client_ = node_handle_.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");
  tele_srv_.request.x = x_;
  tele_srv_.request.y = y_;
  tele_srv_.request.theta = 0;

  // pen client
  ros::ServiceClient pen_client_ = node_handle_.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");
  pen_srv_.request.r = 0;
  pen_srv_.request.g = 0;
  pen_srv_.request.b = 0;
  pen_srv_.request.width = 0;
  pen_srv_.request.off = 1;

  // place turtle at lower left of rectangle
  if(ros::service::exists("/turtle1/set_pen", true) and ros::service::exists("/turtle1/teleport_absolute", true))
  {
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
  }


  ROS_INFO("Successfully launched node.");
}


Control::~Control()
{
}


void Control::controlLoop()
{

  ros::Rate loop_rate(frequency_);

  // states
  int waypoints[4][3] = {{x_, y_, 0}, {x_ + width_, y_, M_PI/2},
                         {x_ + width_, y_ + height_, -M_PI}, {x_, y_ + height_, -M_PI/2}};

  int ctr = 1;

  while(ros::ok())
  {






    ros::spinOnce();
    loop_rate.sleep();
  }
}


bool Control::readParameters(string subscriber_topic)
{
  return (!node_handle_.getParam("subscriber_topic", subscriber_topic)) ? true : false;
}


void Control::poseCallBack(const turtlesim::Pose::ConstPtr& pose_msg)
{
  // pose_.x = pose_msg->x;
  // pose_.y = pose_msg->y;
  pose_ = *pose_msg;
}







int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtle_rect");
  ros::NodeHandle node_handle("~");

  Control control(node_handle);
  //control.controlLoop();

  ros::spin();
  return 0;
}














// end file
