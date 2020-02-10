/// \file
/// \brief Tests low level interface for driving turtlebot
///
/// \author Boston Cleek
/// \date 2/6/20
///
/// PUBLISHES:
///   cmd_vel (geometry_msgs/Twist): Commanded body twist
///   sensor_data (nuturtlebot/SensorData): wheel encoder sesnor data
/// SUBSCRIBES:
///   wheel_cmd (nuturtlebot/WheelCommands): commanded wheel speed -44 to 44
///   joint_states (sensor_msgs/JointState): position and velocity of each wheel


#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

#include <string>
#include <vector>

#include <nuturtlebot/WheelCommands.h>
#include <nuturtlebot/SensorData.h>



/// \brief Stores wheel joint state
struct WheelAxel
{
  std::string name;
  double position = 0.0;
  double velocity = 0.0;
};


/// \brief Helps test the turtlebot wheel speed and sensor msgs
struct TurtlebotHelper
{
  TurtlebotHelper() : left_vel(0), right_vel(0),
                      wheelVelFlag(false), jointStatesFlag(false) {};

  /// \brief Callback for wheel speeds
  /// \param msg - wheel speed message
  void wheelCallBack(const nuturtlebot::WheelCommands::ConstPtr &msg)
  {
    left_vel = msg->left_velocity;
    right_vel = msg->right_velocity;
    wheelVelFlag = true;
  }

  /// \brief Callback for joint states
  /// \param msg - joint name, position, and angular velocity
  void jointCallBack(const sensor_msgs::JointState::ConstPtr &msg)
  {
    // first index if the left wheel
    left_joint.name = msg->name.at(0);
    left_joint.position = msg->position.at(0);
    left_joint.velocity = msg->velocity.at(0);

    right_joint.name = msg->name.at(1);
    right_joint.position = msg->position.at(1);
    right_joint.velocity = msg->velocity.at(1);

    jointStatesFlag = true;
  }


  int left_vel;             // left wheel velocity
  int right_vel;            // right wheel velocity
  bool wheelVelFlag;        // flag for message received

  WheelAxel left_joint;     // left wheel joint
  WheelAxel right_joint;    // right wheel joint
  bool jointStatesFlag;     // flag for joint states message
};




/// \brief Tests the wheel speed drive robot translation
TEST(TurtleInterface, TwistNoRotation)
{
  ros::NodeHandle node_handle;

  TurtlebotHelper turthelper;


  // latched publisher
  ros::Publisher cmd_pub = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
  ros::Subscriber wheel_sub = node_handle.subscribe("/wheel_cmd", 1, &TurtlebotHelper::wheelCallBack, &turthelper);

  geometry_msgs::Twist cmd;
  cmd.linear.x = 0.1;
  cmd.linear.y = 0;
  cmd.linear.z = 0;
  cmd.angular.x = 0;
  cmd.angular.y = 0;
  cmd.angular.z = 0;


  while(!turthelper.wheelVelFlag)
  {
    ros::spinOnce();
    cmd_pub.publish(cmd);
  }

  ASSERT_EQ(turthelper.left_vel, 21);
  ASSERT_EQ(turthelper.right_vel, 21);
}



/// \brief Tests the wheel speed drive robot rotation
TEST(TurtleInterface, TwistNoTranslation)
{
  ros::NodeHandle node_handle;

  TurtlebotHelper turthelper;


  // latched publisher
  ros::Publisher cmd_pub = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
  ros::Subscriber wheel_sub = node_handle.subscribe("/wheel_cmd", 1, &TurtlebotHelper::wheelCallBack, &turthelper);

  geometry_msgs::Twist cmd;
  cmd.linear.x = 0;
  cmd.linear.y = 0;
  cmd.linear.z = 0;
  cmd.angular.x = 0;
  cmd.angular.y = 0;
  cmd.angular.z = 1;


  while(!turthelper.wheelVelFlag)
  {
    ros::spinOnce();
    cmd_pub.publish(cmd);
  }

  ASSERT_EQ(turthelper.left_vel, -17);
  ASSERT_EQ(turthelper.right_vel, 17);
}


/// \brief Tests translation and rotation wheel speeds
TEST(TurtleInterface, TwistTransRot)
{
  ros::NodeHandle node_handle;

  TurtlebotHelper turthelper;


  // latched publisher
  ros::Publisher cmd_pub = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
  ros::Subscriber wheel_sub = node_handle.subscribe("/wheel_cmd", 1, &TurtlebotHelper::wheelCallBack, &turthelper);

  geometry_msgs::Twist cmd;
  cmd.linear.x = 0.01;
  cmd.linear.y = 0;
  cmd.linear.z = 0;
  cmd.angular.x = 0;
  cmd.angular.y = 0;
  cmd.angular.z = 1;


  while(!turthelper.wheelVelFlag)
  {
    ros::spinOnce();
    cmd_pub.publish(cmd);
  }

  ASSERT_EQ(turthelper.left_vel, -15);
  ASSERT_EQ(turthelper.right_vel, 19);
}



/// \brief Tests translation and rotation wheel speeds
TEST(TurtleInterface, Sensort2JointState)
{
  ros::NodeHandle node_handle;

  TurtlebotHelper turthelper;


  // latched publisher
  ros::Publisher sensor_pub = node_handle.advertise<nuturtlebot::SensorData>("/sensor_data", 1, true);
  ros::Subscriber wheel_sub = node_handle.subscribe("/joint_states", 1, &TurtlebotHelper::jointCallBack, &turthelper);


  nuturtlebot::SensorData sensor_data;
  sensor_data.left_encoder = 100;
  sensor_data.right_encoder = 100;


  while(!turthelper.jointStatesFlag)
  {
    ros::spinOnce();
    sensor_pub.publish(sensor_data);
  }

  ASSERT_NEAR(turthelper.left_joint.position, 0.15339807878856412, 1e-3);
  ASSERT_NEAR(turthelper.right_joint.position, 0.15339807878856412,1e-3);

  ASSERT_NEAR(turthelper.left_joint.velocity, 0.15339807878856412, 1e-3);
  ASSERT_NEAR(turthelper.right_joint.velocity, 0.15339807878856412,1e-3);
}






int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "turtle_interface_test");

  ROS_INFO("Successfully launched turtle_interface_test node");

  return RUN_ALL_TESTS();
}


















// end file
