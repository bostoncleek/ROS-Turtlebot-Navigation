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
  ASSERT_NEAR(twist.vx, 0.2, 1e-6);
  ASSERT_NEAR(twist.vy, 0, 1e-6);

  // pure rotation
  vel.ul = -10;
  vel.ur = 10;

  twist = drive.wheelsToTwist(vel);

  ASSERT_NEAR(twist.w, 0.4, 1e-6);
  ASSERT_NEAR(twist.vx, 0, 1e-6);
  ASSERT_NEAR(twist.vy, 0, 1e-6);


  // translationa and rotation
  vel.ul = 0;
  vel.ur = 10;

  twist = drive.wheelsToTwist(vel);

  ASSERT_NEAR(twist.w, 0.2, 1e-6);
  ASSERT_NEAR(twist.vx, 0.1, 1e-6);
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
  double wheel_angle = rigid2d::PI/30;
  vel = drive.updateOdometry(wheel_angle, wheel_angle);

  pose = drive.pose();


  ASSERT_NEAR(vel.ul, 0.10472 , 1e-3);
  ASSERT_NEAR(vel.ur, 0.10472 , 1e-3);

  ASSERT_NEAR(pose.theta, 0, 1e-3);
  ASSERT_NEAR(pose.x, 0.0020944, 1e-3);
  ASSERT_NEAR(pose.y, 0, 1e-3);


  // std::cout << "------------------" << std::endl;
  // std::cout << vel.ul << " " << vel.ur << std::endl;
  // std::cout << "------------------" << std::endl;
  // std::cout << "------------------" << std::endl;
  // std::cout << pose.theta << " " << pose.x << " " << pose.y << std::endl;
  // std::cout << "------------------" << std::endl;
}


/// brief Test odometry in a straight line
TEST(DiffDriveTest, NoMovementOdom)
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
  double wheel_angle = 0;
  vel = drive.updateOdometry(wheel_angle, wheel_angle);

  pose = drive.pose();


  ASSERT_NEAR(vel.ul, 0.0, 1e-3);
  ASSERT_NEAR(vel.ur, 0.0, 1e-3);

  ASSERT_NEAR(pose.theta, 0.0, 1e-3);
  ASSERT_NEAR(pose.x, 0.0, 1e-3);
  ASSERT_NEAR(pose.y, 0.0, 1e-3);

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
  double left_wheel = -rigid2d::PI/30;
  double right_wheel = rigid2d::PI/30;


  vel = drive.updateOdometry(left_wheel, right_wheel);

  pose = drive.pose();


  ASSERT_NEAR(vel.ul, -0.10472, 1e-3);
  ASSERT_NEAR(vel.ur, 0.10472, 1e-3);

  ASSERT_NEAR(pose.theta, 0.00418879, 1e-3);
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
  double right_wheel = rigid2d::PI/30;


  vel = drive.updateOdometry(left_wheel, right_wheel);

  pose = drive.pose();


  ASSERT_NEAR(vel.ul, 0, 1e-3);
  ASSERT_NEAR(vel.ur, 0.10472, 1e-3);

  ASSERT_NEAR(pose.theta, 0.0020944, 1e-3);
  ASSERT_NEAR(pose.x, 0.0010472, 1e-3);
  ASSERT_NEAR(pose.y, 0, 1e-3);

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
  rigid2d::WheelEncoders encoder;

  rigid2d::Pose pose;
  pose.theta = 0.0;
  pose.x = 0.0;
  pose.y = 0.0;

  double wheel_radius = 0.02;
  double wheel_base = 1.0;

  rigid2d::DiffDrive drive(pose, wheel_base, wheel_radius);

  cmd.w = 0.0;
  cmd.vx = 0.01;
  cmd.vy = 0.0;

  // apply forward twist
  drive.feedforward(cmd);

  pose = drive.pose();

  encoder = drive.getEncoders();

  ASSERT_NEAR(pose.theta, 0, 1e-3);
  ASSERT_NEAR(pose.x, 0.01, 1e-3);
  ASSERT_NEAR(pose.y, 0, 1e-3);

  // std::cout << "------------------" << std::endl;
  // std::cout << encoder.left << " " << encoder.right << std::endl;
  // std::cout << "------------------" << std::endl;
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

  cmd.w = rigid2d::PI/10;
  cmd.vx = 0.0;
  cmd.vy = 0.0;

  // apply forward twist
  drive.feedforward(cmd);

  pose = drive.pose();


  ASSERT_NEAR(pose.theta, 0.314159, 1e-3);
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

  cmd.w = rigid2d::PI/10;
  cmd.vx = 0.01;
  cmd.vy = 0.0;

  // apply forward twist
  drive.feedforward(cmd);

  pose = drive.pose();


  ASSERT_NEAR(pose.theta, 0.314159, 1e-3);
  ASSERT_NEAR(pose.x, 0.00983632, 1e-3);
  ASSERT_NEAR(pose.y, 0.00155792, 1e-3);

  // std::cout << "------------------" << std::endl;
  // std::cout << pose.theta << " " << pose.x << " " << pose.y << std::endl;
  // std::cout << "------------------" << std::endl;
}


TEST(DiffDriveTestm, FeedForwardUpdateOdom)
{
  rigid2d::Twist2D cmd;

  rigid2d::Pose pose_1;
  rigid2d::Pose pose_2;

  rigid2d::WheelEncoders encoder_1;
  rigid2d::WheelEncoders encoder_2;

  rigid2d::WheelVelocities vel_1;
  rigid2d::WheelVelocities vel_2;


  pose_1.theta = 0.0;
  pose_1.x = 0.0;
  pose_1.y = 0.0;

  pose_2 = pose_1;

  double wheel_radius = 0.02;
  double wheel_base = 1.0;

  rigid2d::DiffDrive drive_1(pose_1, wheel_base, wheel_radius);
  rigid2d::DiffDrive drive_2(pose_2, wheel_base, wheel_radius);


  cmd.w = 0.0;
  cmd.vx = 0.01;
  cmd.vy = 0.0;

  // apply forward twist
  drive_1.feedforward(cmd);

  pose_1 = drive_1.pose();

  encoder_1 = drive_1.getEncoders();

  vel_1 = drive_1.wheelVelocities();

  // std::cout << "feedforward" << std::endl;
  // std::cout << "------------------" << std::endl;
  // std::cout << "pose" << std::endl;
  // std::cout << pose_1.theta << " " << pose_1.x << " " << pose_1.y << std::endl;
  // std::cout << "------------------" << std::endl;
  // std::cout << "encoders" << std::endl;
  // std::cout << encoder_1.left << " " << encoder_1.right << std::endl;
  // std::cout << "------------------" << std::endl;
  // std::cout << "wheel vel" << std::endl;
  // std::cout << vel_1.ul << " " << vel_1.ur << std::endl;
  // std::cout << "------------------" << std::endl;


  vel_2 = drive_2.updateOdometry(encoder_1.left, encoder_1.right);

  pose_2 = drive_2.pose();

  encoder_2 = drive_2.getEncoders();

  vel_2 = drive_2.wheelVelocities();


  // std::cout << "updateOdometry" << std::endl;
  // std::cout << "------------------" << std::endl;
  // std::cout << "pose" << std::endl;
  // std::cout << pose_2.theta << " " << pose_2.x << " " << pose_2.y << std::endl;
  // std::cout << "------------------" << std::endl;
  // std::cout << "encoders" << std::endl;
  // std::cout << encoder_2.left << " " << encoder_2.right << std::endl;
  // std::cout << "------------------" << std::endl;
  // std::cout << "wheel vel" << std::endl;
  // std::cout << vel_2.ul << " " << vel_2.ur << std::endl;
  // std::cout << "------------------" << std::endl;


  ASSERT_NEAR(pose_1.theta, pose_2.theta, 1e-3);
  ASSERT_NEAR(pose_1.x, pose_2.x, 1e-3);
  ASSERT_NEAR(pose_1.y, pose_2.y, 1e-3);

  ASSERT_NEAR(encoder_1.left, encoder_2.left, 1e-3);
  ASSERT_NEAR(encoder_1.right, encoder_2.right, 1e-3);

  ASSERT_NEAR(vel_1.ul, vel_2.ul, 1e-3);
  ASSERT_NEAR(vel_1.ur, vel_2.ur, 1e-3);
}












// end file
