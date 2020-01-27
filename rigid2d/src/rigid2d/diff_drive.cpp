/// \file
/// \brief implementation of diff_drive methods in rigid2d namespace

#include <iostream>
#include <stdexcept>
#include "rigid2d/diff_drive.hpp"


namespace rigid2d
{

DiffDrive::DiffDrive()
{
  theta = 0.0;
  x = 0.0;
  y = 0.0;

  // fixed geometry;
  wheel_base = 0.1;
  wheel_radius = 0.02;

  // set wheel encoder angles
  left_curr = 0.0;
  right_curr = 0.0;

  // assume robot starts still
  ul = 0.0;
  ur = 0.0;

  // assume dt = 1
  dt = 1;
}


DiffDrive::DiffDrive(const Pose &pose, double wheel_base, double wheel_radius)
{
  this->theta = pose.theta;
  this->x = pose.x;
  this->y = pose.y;
  this->wheel_base = wheel_base;
  this->wheel_radius = wheel_radius;

  // set wheel encoder angles
  left_curr = 0.0;
  right_curr = 0.0;

  // assume robot starts still
  ul = 0.0;
  ur = 0.0;

  // assume dt = 1
  dt = 1;
}


WheelVelocities DiffDrive::twistToWheels(const Twist2D &twist) const
{

  // Using Eq. 1 multiply H by the twist.
  // See /doc for more details.


  double d = wheel_base / 2;

  WheelVelocities vel;

  vel.ul = (1 / wheel_radius) * (-d*twist.w + twist.vx);
  vel.ur = (1 / wheel_radius) * (d*twist.w + twist.vx);

  if (twist.vy != 0)
    throw std::invalid_argument("Twist cannot have y velocity component");

  return vel;
}


Twist2D DiffDrive::wheelsToTwist(const WheelVelocities &vel) const
{

  // Using Eq. 2 multiply the pseudo inverse of H by the wheel velocities.
  // See /doc for more details.

  double d = 1 / wheel_base;

  Twist2D twist;
  twist.w = wheel_radius * d * (vel.ur - vel.ul);
  twist.vx = wheel_radius * 0.5 * (vel.ul + vel.ur);
  twist.vy = 0.0;

  return twist;

}


WheelVelocities DiffDrive::updateOdometry(double left, double right)
{
  WheelVelocities vel;

  // wheel velocities are changes in wheel angles
  vel.ul = normalize_angle_PI(left - left_curr);
  vel.ur = normalize_angle_PI(right - right_curr);

  // vel.ul = left - left_curr;
  // vel.ur = right - right_curr;

  // update wheel velocities
  this->ul = vel.ul;
  this->ur = vel.ur;

  // update wheel angles
  left_curr = normalize_angle_PI(left);
  right_curr = normalize_angle_PI(right);

  // left_curr = left;
  // right_curr = right;



  // body frame twist given wheel velocities
  Twist2D Vb = wheelsToTwist(vel);

  // std::cout << Vb.w << " " << Vb.vx << "\n";

  // itegrate twist
  Transform2D Tb_bprime;
  Tb_bprime = Tb_bprime.integrateTwist(Vb);


  // pose is transform form world to b prime
  Vector2D v;
  v.x = this->x;
  v.y = this->y;
  Transform2D Twb(v, this->theta);

  // update pose to b prime
  Transform2D Tw_bprime = Twb * Tb_bprime;

  // world to robot
  TransformData2D Twr = Tw_bprime.displacement();

  // update pose
  this->theta = normalize_angle_PI(Twr.theta);
  // this->theta = Twr.theta;
  this->x = Twr.x;
  this->y = Twr.y;

  return vel;
}


void DiffDrive::feedforward(const Twist2D &cmd)
{

  // wheel velocities to achieve the cmd
  WheelVelocities vel = twistToWheels(cmd);


  // update wheel velocities
  ul = normalize_angle_PI(vel.ul);
  ur = normalize_angle_PI(vel.ur);

  // update encoder readings
  left_curr = normalize_angle_PI(left_curr + vel.ul);
  right_curr = normalize_angle_PI(right_curr + vel.ur);

  // left_curr = left_curr + vel.ul;
  // right_curr = right_curr + vel.ur;


  // itegrate twist
  Transform2D Tb_bprime;
  Tb_bprime = Tb_bprime.integrateTwist(cmd);


  // pose is transform form world to b prime
  Vector2D v;
  v.x = this->x;
  v.y = this->y;
  Transform2D Twb(v, this->theta);

  // update pose to b prime
  Transform2D Tw_bprime = Twb * Tb_bprime;

  // world to robot
  TransformData2D Twr = Tw_bprime.displacement();

  // update pose
  this->theta = normalize_angle_PI(Twr.theta);
  // this->theta = Twr.theta;

  this->x = Twr.x;
  this->y = Twr.y;
}


Pose DiffDrive::pose() const
{
  Pose pose;
  pose.theta = normalize_angle_PI(this->theta);
  pose.x = this->x;
  pose.y = this->y;

  return pose;
}


WheelVelocities DiffDrive::wheelVelocities() const
{
  WheelVelocities vel;
  vel.ul = this->ul;
  vel.ur = this->ur;

  return vel;
}


void DiffDrive::reset(Pose ps)
{
  theta = ps.theta;
  x = ps.x;
  y = ps.y;

  // set wheel encoder angles
  left_curr = 0.0;
  right_curr = 0.0;

  // assume robot starts still
  ul = 0.0;
  ur = 0.0;
}


WheelEncoders DiffDrive::getEncoders() const
{
  WheelEncoders encoder;
  encoder.left = this->left_curr;
  encoder.right = this->right_curr;

  return encoder;
}

// void DiffDrive::scaleTwist(double dt)
// {
//   this->dt = dt;
// }




} // end namespace








// end file
