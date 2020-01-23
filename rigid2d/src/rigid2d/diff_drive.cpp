/// \file
/// \brief implementation of diff_drive methods in rigid2d namespace

#include <iostream>
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
}


WheelVelocities DiffDrive::twistToWheels(const Twist2D &twist)
{
  double d = wheel_base / 2.0;

  WheelVelocities vel;
  vel.ul = (1.0 / wheel_radius)*(-d * twist.w + twist.vx);
  vel.ur = (1.0 / wheel_radius)*(d * twist.w + twist.vx);

  return vel;
}


Twist2D DiffDrive::wheelsToTwist(const WheelVelocities &vel)
{
  double d = wheel_base / 2.0;

  Twist2D twist;
  twist.w = (vel.ur - vel.ul) / (2*d);
  twist.vx = (vel.ul + vel.ur) / 2.0;
  twist.vy = 0.0;

  return twist;

}


WheelVelocities DiffDrive::updateOdometry(double left, double right)
{
  WheelVelocities vel;

  // wheel velocities are changes in wheel angles
  vel.ul = left - left_curr;
  vel.ur = right - right_curr;

  this->ul = vel.ul;
  this->ur = vel.ur;

  // body frame twist given wheel velocities
  Twist2D Vb = wheelsToTwist(vel);

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
  this->theta = Twr.theta;
  this->x = Twr.x;
  this->y = Twr.y;


  // update wheel angles
  left_curr = left;
  right_curr = right;

  return vel;
}


void DiffDrive::feedforward(const Twist2D &cmd)
{
  // wheel velocities to achieve the cmd
  WheelVelocities vel = twistToWheels(cmd);

  // BUG: Should I update these ????
  // update encoder readings
  left_curr += vel.ul;
  right_curr += vel.ur;

  // BUG: update these ???
  ul = vel.ul;
  ur = vel.ur;

  // body frame twist given wheel velocities
  // Twist2D Vb = wheelsToTwist(vel);

  // itegrate twist
  Transform2D Tb_bprime;
  Tb_bprime = Tb_bprime.integrateTwist(cmd);



  // std::cout << "------------------" << std::endl;
  // std::cout << "Tbb'" << std::endl;
  // std::cout << Tb_bprime;
  // std::cout << "------------------" << std::endl;


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
  this->theta = Twr.theta;
  this->x = Twr.x;
  this->y = Twr.y;
}


Pose DiffDrive::pose()
{
  // BUG: reset wheel encoders and speeds ???

  Pose pose;
  pose.theta = this->theta;
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
}



} // end namespace








// end file
