#include <ros/ros.h>
#include <ros/console.h>
#include <turtlesim/SetPen.h>
#include <turtlesim/TeleportAbsolute.h>
//#include <vector>

using std::cout;
using std::endl;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtle_rect");
  ros::NodeHandle node_handle("~");

  int x, y, width, height, trans_vel, rot_vel, frequency;
  node_handle.getParam("/x", x);
  node_handle.getParam("/y", y);
  node_handle.getParam("/width", width);
  node_handle.getParam("/height", height);
  node_handle.getParam("/trans_vel", trans_vel);
  node_handle.getParam("/rot_vel", rot_vel);
  node_handle.getParam("/frequency", frequency);

  ROS_INFO("x: %d", x);
  ROS_INFO("y: %d", y);
  ROS_INFO("width: %d", width);
  ROS_INFO("height: %d", height);
  ROS_INFO("trans_vel: %d", trans_vel);
  ROS_INFO("rot_vel: %d", rot_vel);
  ROS_INFO("frequency: %d", frequency);


  ros::ServiceClient tele_client = node_handle.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");
  turtlesim::TeleportAbsolute tele_srv;
  tele_srv.request.x = x;
  tele_srv.request.y = y;
  tele_srv.request.theta = 0;


  ros::ServiceClient pen_client = node_handle.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");
  turtlesim::SetPen pen_srv;
  pen_srv.request.r = 0;
  pen_srv.request.g = 0;
  pen_srv.request.b = 0;
  pen_srv.request.width = 0;
  pen_srv.request.off = 1;

  if(ros::service::exists("/turtle1/set_pen", true) and ros::service::exists("/turtle1/teleport_absolute", true))
  {
    // cout << "called" << endl;
    pen_client.call(pen_srv);     // turn off pen
    tele_client.call(tele_srv);   // teleport

    // turn pen back on
    pen_srv.request.r = 255;
    pen_srv.request.g = 255;
    pen_srv.request.b = 255;
    pen_srv.request.width = 2;
    pen_srv.request.off = 0;
  }




  ros::spin();
  return 0;
}














// end file
