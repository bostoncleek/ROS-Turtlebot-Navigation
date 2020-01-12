/// \file
/// \brief implementation of methods in rigid2d namespace

#include "rigid2d.hpp"
#include <iostream>

namespace rigid2d
{


std::ostream & operator<<(std::ostream & os, const Vector2D & v)
{
  // os << "x component: " <<  v.x << " y component: " << v.y << '\n';
  os << "[" <<  v.x << " " << v.y <<"]\n";
  return os;
}


std::istream & operator>>(std::istream & is, Vector2D & v)
{
  std::cout << "Enter x component" << std::endl;
  is >> v.x;

  std::cout << "Enter y component" << std::endl;
  is >> v.y;

  return is;
}


std::ostream & operator<<(std::ostream & os, const Twist2D & twist)
{
  // os << "angular: " << twist.w << " x component: " <<  twist.vx << " y component: " << twist.vy << '\n';
  os << "[" << twist.w << " " <<  twist.vx << " " << twist.vy << "]\n";
  return os;
}


std::istream & operator>>(std::istream & is, Twist2D & twist)
{
  std::cout << "Enter angular component" << std::endl;
  is >> twist.w;

  std::cout << "Enter linear velocity x component" << std::endl;
  is >> twist.vx;

  std::cout << "Enter linear velocity y component" << std::endl;
  is >> twist.vy;

  return is;
}


NormalVec2D normalize(const Vector2D & v)
{
  NormalVec2D nv;

  double mag = std::sqrt(std::pow(v.x, 2) + std::pow(v.y, 2));
  nv.nx = v.x / mag;
  nv.ny = v.y / mag;

  return nv;
}


// public


Transform2D::Transform2D()
{
  theta = 0.0;
  ctheta = 1.0;
  stheta = 0.0;
  x = 0.0;
  y = 0.0;
}


Transform2D::Transform2D(const Vector2D & trans)
{
  theta = 0.0;
  ctheta = 0.0;
  stheta = 0.0;
  x = trans.x;
  y = trans.y;
}


Transform2D::Transform2D(double radians)
{
  theta = radians;
  ctheta = std::cos(theta);
  stheta = std::sin(theta);
  x = 0.0;
  y = 0.0;
}


Transform2D::Transform2D(const Vector2D & trans, double radians)
{
  theta = radians;
  ctheta = std::cos(theta);
  stheta = std::sin(theta);
  x = trans.x;
  y = trans.y;
}


Vector2D Transform2D::operator()(Vector2D v) const
{
  Vector2D v_new;
  v_new.x = ctheta * v.x - stheta * v.y;
  v_new.y = stheta * v.x + ctheta * v.y;

  return v_new;
}


Transform2D Transform2D::inv() const
{
  // create new transform
  Transform2D trans2d(theta, ctheta, stheta, x, y);

  // R^T flip sign in sin
  trans2d.stheta = -1.0 * stheta;

  // p' = -R^T * p
  trans2d.x = -(trans2d.ctheta * x - trans2d.stheta * y);
  trans2d.y = -(trans2d.stheta * x + trans2d.ctheta * y);

  return trans2d;
}


Twist2D Transform2D::operator()(Twist2D twist) const
{
  Twist2D twist_new;
  twist_new.w = twist.w;

  twist_new.vx = twist.vx * ctheta - twist.vy * stheta + twist.w * y;

  twist_new.vy = twist.vy * ctheta + twist.vx * stheta + twist.w * x;

  return twist_new;
}



// private

Transform2D::Transform2D(double theta, double ctheta, double stheta, double x, double y)
{
  this->theta = theta;
  this->ctheta = ctheta;
  this->stheta = stheta;
  this->x = x;
  this->y = y;
}


Transform2D & Transform2D::operator*=(const Transform2D & rhs)
{
  // // multiply rotations
  // double c_new = ctheta * rhs.ctheta - stheta * rhs.stheta;
  // double s_new = stheta * rhs.ctheta + ctheta * rhs.stheta;
  //
  // // multiply rotation by translation
  // double x_new = ctheta * rhs.x - stheta * rhs.y + x;
  // double y_new = stheta * rhs.x + ctheta * rhs.y + y;
  //
  // // set new transform
  // ctheta = c_new;
  // stheta = s_new;
  // theta = std::acos(ctheta);
  //
  // x = x_new;
  // y = y_new;

  x = ctheta * rhs.x - stheta * rhs.y + x;
  y = stheta * rhs.x + ctheta * rhs.y + y;
  theta = std::acos(ctheta * rhs.ctheta - stheta * rhs.stheta);
  ctheta = std::cos(theta);
  stheta = std::sin(theta);

  return *this;
}


// end class


std::ostream & operator<<(std::ostream & os, const Transform2D & tf)
{
  os << "theta (degrees): " << rad2deg(tf.theta) << " x: " << tf.x << " y: " << tf.y << "\n";
  return os;
}


std::istream & operator>>(std::istream & is, Transform2D & tf)
{
  double deg_angle;
  Vector2D v;

  std::cout << "Enter angle in degrees" << std::endl;
  is >> deg_angle;

  std::cout << "Enter x component" << std::endl;
  is >> v.x;

  std::cout << "Enter y component" << std::endl;
  is >> v.y;

  Transform2D tf_temp(v, deg2rad(deg_angle));
  tf = tf_temp;

  return is;
}


Transform2D operator*(Transform2D lhs, const Transform2D & rhs)
{
  // get access to lhs private members
  return lhs.operator*=(rhs);
}



} // end namespace












// end file
