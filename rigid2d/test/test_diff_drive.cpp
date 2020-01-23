/// \file
/// \brief unit tests for rigid2d library

#include <gtest/gtest.h>
#include <iostream>
#include <sstream>
#include <cmath>

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"


/// \brief Tests converting a twist to wheel velocities
TEST(DiffDriveTest, TwistToWheels)
{
  rigid2d::WheelVelocities vel;
  rigid2d::Twist2D twist;

  rigid2d::Pose pose;
  pose.theta = 0.0;
  pose.x = 0.0;
  pose.y = 0.0;

  double wheel_radius = 0.02;
  double wheel_base = 1.0;

  rigid2d::DiffDrive drive(pose, wheel_base, wheel_radius);

  // Pure rotation
  twist.w = 1.0;
  twist.vx = 0.0;
  twist.vy = 0.0;

  vel = drive.twistToWheels(twist);

  ASSERT_NEAR(vel.ul, -25, 1e-6);
  ASSERT_NEAR(vel.ur, 25, 1e-6);


  // Pure translation
  twist.w = 0.0;
  twist.vx = 1.0;
  twist.vy = 0.0;

  vel = drive.twistToWheels(twist);

  ASSERT_NEAR(vel.ul, 50, 1e-6);
  ASSERT_NEAR(vel.ur, 50, 1e-6);


  // rotation and translation
  twist.w = 1.0;
  twist.vx = 1.0;
  twist.vy = 0.0;

  vel = drive.twistToWheels(twist);

  ASSERT_NEAR(vel.ul, 25, 1e-6);
  ASSERT_NEAR(vel.ur, 75, 1e-6);

  // std::cout << "------------------" << std::endl;
  // std::cout << vel.ul << " " << vel.ur << std::endl;
  // std::cout << "------------------" << std::endl;
}


/// \brief Tests converting wheel velocities to a twist
TEST(DiffDriveTest, WheelsToTwist)
{
  rigid2d::WheelVelocities vel;
  rigid2d::Twist2D twist;

  rigid2d::Pose pose;
  pose.theta = 0.0;
  pose.x = 0.0;
  pose.y = 0.0;

  double wheel_radius = 0.02;
  double wheel_base = 1.0;

  rigid2d::DiffDrive drive(pose, wheel_base, wheel_radius);

  // pure translation
  vel.ul = 10;
  vel.ur = 10;

  twist = drive.wheelsToTwist(vel);

  ASSERT_NEAR(twist.w, 0, 1e-6);
  ASSERT_NEAR(twist.vx, 10, 1e-6);
  ASSERT_NEAR(twist.vy, 0, 1e-6);

  // pure rotation
  vel.ul = -10;
  vel.ur = 10;

  twist = drive.wheelsToTwist(vel);

  ASSERT_NEAR(twist.w, 20, 1e-6);
  ASSERT_NEAR(twist.vx, 0, 1e-6);
  ASSERT_NEAR(twist.vy, 0, 1e-6);


  // translationa and rotation
  vel.ul = 0;
  vel.ur = 10;

  twist = drive.wheelsToTwist(vel);

  ASSERT_NEAR(twist.w, 10, 1e-6);
  ASSERT_NEAR(twist.vx, 5, 1e-6);
  ASSERT_NEAR(twist.vy, 0, 1e-6);

  // std::cout << "------------------" << std::endl;
  // std::cout << twist.w << " " << twist.vx << " " << twist.vy << std::endl;
  // std::cout << "------------------" << std::endl;
}


/// brief Test odometry in a straight line
TEST(DiffDriveTest, PureTranslationOdom)
{
  rigid2d::WheelVelocities vel;

  rigid2d::Pose pose;
  pose.theta = 0.0;
  pose.x = 0.0;
  pose.y = 0.0;

  double wheel_radius = 0.02;
  double wheel_base = 1.0;

  rigid2d::DiffDrive drive(pose, wheel_base, wheel_radius);

  // both wheels rotate 2pi
  double wheel_angle = 2*rigid2d::PI;
  vel = drive.updateOdometry(wheel_angle, wheel_angle);

  pose = drive.pose();


  ASSERT_NEAR(vel.ul, 6.28319, 1e-3);
  ASSERT_NEAR(vel.ur, 6.28319, 1e-3);

  ASSERT_NEAR(pose.theta, 0, 1e-3);
  ASSERT_NEAR(pose.x, 6.28319, 1e-3);
  ASSERT_NEAR(pose.y, 0, 1e-3);


  // std::cout << "------------------" << std::endl;
  // std::cout << vel.ul << " " << vel.ur << std::endl;
  // std::cout << "------------------" << std::endl;
  // std::cout << "------------------" << std::endl;
  // std::cout << pose.theta << " " << pose.x << " " << pose.y << std::endl;
  // std::cout << "------------------" << std::endl;
}


/// \brief Test odometry in a pure rotation
TEST(DiffDriveTest, PureRotationOdom)
{
  rigid2d::WheelVelocities vel;

  rigid2d::Pose pose;
  pose.theta = 0.0;
  pose.x = 0.0;
  pose.y = 0.0;

  double wheel_radius = 0.02;
  double wheel_base = 1.0;

  rigid2d::DiffDrive drive(pose, wheel_base, wheel_radius);

  // both wheels rotate 2pi
  double left_wheel = -rigid2d::PI/4;
  double right_wheel = rigid2d::PI/4;


  vel = drive.updateOdometry(left_wheel, right_wheel);

  pose = drive.pose();


  ASSERT_NEAR(vel.ul, -0.785398, 1e-3);
  ASSERT_NEAR(vel.ur, 0.785398, 1e-3);

  ASSERT_NEAR(pose.theta, 1.5708, 1e-3);
  ASSERT_NEAR(pose.x, 0, 1e-3);
  ASSERT_NEAR(pose.y, 0, 1e-3);

  // std::cout << "------------------" << std::endl;
  // std::cout << vel.ul << " " << vel.ur << std::endl;
  // std::cout << "------------------" << std::endl;
  // std::cout << "------------------" << std::endl;
  // std::cout << pose.theta << " " << pose.x << " " << pose.y << std::endl;
  // std::cout << "------------------" << std::endl;
}


/// brief Test odometry in a translation and rotation
TEST(DiffDriveTest, TransRotOdom)
{
  rigid2d::WheelVelocities vel;

  rigid2d::Pose pose;
  pose.theta = 0.0;
  pose.x = 0.0;
  pose.y = 0.0;

  double wheel_radius = 0.02;
  double wheel_base = 1.0;

  rigid2d::DiffDrive drive(pose, wheel_base, wheel_radius);

  // both wheels rotate 2pi
  double left_wheel = 0;
  double right_wheel = rigid2d::PI/4;


  vel = drive.updateOdometry(left_wheel, right_wheel);

  pose = drive.pose();


  ASSERT_NEAR(vel.ul, 0, 1e-3);
  ASSERT_NEAR(vel.ur, 0.785398, 1e-3);

  ASSERT_NEAR(pose.theta, 0.785398, 1e-3);
  ASSERT_NEAR(pose.x, 0.353553, 1e-3);
  ASSERT_NEAR(pose.y, 0.146447, 1e-3);

  // std::cout << "------------------" << std::endl;
  // std::cout << vel.ul << " " << vel.ur << std::endl;
  // std::cout << "------------------" << std::endl;
  // std::cout << "------------------" << std::endl;
  // std::cout << pose.theta << " " << pose.x << " " << pose.y << std::endl;
  // std::cout << "------------------" << std::endl;
}



/// brief Test feedforward in a straight line
TEST(DiffDriveTest, StraightLineFeedForward)
{
  rigid2d::Twist2D cmd;

  rigid2d::Pose pose;
  pose.theta = 0.0;
  pose.x = 0.0;
  pose.y = 0.0;

  double wheel_radius = 0.02;
  double wheel_base = 1.0;

  rigid2d::DiffDrive drive(pose, wheel_base, wheel_radius);

  cmd.w = 0.0;
  cmd.vx = 1.0;
  cmd.vy = 0.0;

  // apply forward twist
  drive.feedforward(cmd);

  pose = drive.pose();


  ASSERT_NEAR(pose.theta, 0, 1e-3);
  ASSERT_NEAR(pose.x, 1, 1e-3);
  ASSERT_NEAR(pose.y, 0, 1e-3);


  // std::cout << "------------------" << std::endl;
  // std::cout << pose.theta << " " << pose.x << " " << pose.y << std::endl;
  // std::cout << "------------------" << std::endl;
}


/// \brief Test feedforward in pure rotation
TEST(DiffDriveTest, RotationFeedForward)
{
  rigid2d::Twist2D cmd;

  rigid2d::Pose pose;
  pose.theta = 0.0;
  pose.x = 0.0;
  pose.y = 0.0;

  double wheel_radius = 0.02;
  double wheel_base = 1.0;

  rigid2d::DiffDrive drive(pose, wheel_base, wheel_radius);

  cmd.w = rigid2d::PI/4;
  cmd.vx = 0.0;
  cmd.vy = 0.0;

  // apply forward twist
  drive.feedforward(cmd);

  pose = drive.pose();


  ASSERT_NEAR(pose.theta, 0.785398, 1e-3);
  ASSERT_NEAR(pose.x, 0, 1e-3);
  ASSERT_NEAR(pose.y, 0, 1e-3);

  // std::cout << "------------------" << std::endl;
  // std::cout << pose.theta << " " << pose.x << " " << pose.y << std::endl;
  // std::cout << "------------------" << std::endl;
}



/// brief Test feedforward in translation and rotation
TEST(DiffDriveTest, TransRotFeedForward)
{
  rigid2d::Twist2D cmd;

  rigid2d::Pose pose;
  pose.theta = 0.0;
  pose.x = 0.0;
  pose.y = 0.0;

  double wheel_radius = 0.02;
  double wheel_base = 1.0;

  rigid2d::DiffDrive drive(pose, wheel_base, wheel_radius);

  cmd.w = rigid2d::PI/4;
  cmd.vx = 1.0;
  cmd.vy = 0.0;

  // apply forward twist
  drive.feedforward(cmd);

  pose = drive.pose();


  ASSERT_NEAR(pose.theta, 0.785398, 1e-3);
  ASSERT_NEAR(pose.x, 0.900316, 1e-3);
  ASSERT_NEAR(pose.y, 0.372923, 1e-3);

  // std::cout << "------------------" << std::endl;
  // std::cout << pose.theta << " " << pose.x << " " << pose.y << std::endl;
  // std::cout << "------------------" << std::endl;
}















// end file
