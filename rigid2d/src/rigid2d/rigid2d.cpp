/// \file
/// \brief implementation of methods in rigid2d namespace

#include <iostream>
#include "rigid2d/rigid2d.hpp"


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


Vector2D operator+(Vector2D v1, const Vector2D & v2)
{
  return v1 += v2;
}


Vector2D operator-(Vector2D v1, const Vector2D &v2)
{
  return v1 -= v2;
}


Vector2D operator*(Vector2D v, const double scalar)
{
  return v *= scalar;
}


Vector2D operator*(const double scalar, Vector2D v)
{
  return v *= scalar;
}


double length(const Vector2D &v)
{
  return std::sqrt(std::pow(v.x, 2) + std::pow(v.y, 2));
}


double distance(const Vector2D &v1, const Vector2D &v2)
{
  return std::sqrt(std::pow(v1.x - v2.x, 2) + std::pow(v1.y - v2.y, 2));
}


double angle(const Vector2D &v1, const Vector2D &v2)
{
  // dot product
  double dot = v1.x * v2.x + v1.y * v2.y;

  return std::acos(dot / (length(v1) * length(v2)));
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
  v_new.x = ctheta * v.x - stheta * v.y + x;
  v_new.y = stheta * v.x + ctheta * v.y + y;

  return v_new;
}


Transform2D Transform2D::inv() const
{
  // create new transform
  Transform2D trans2d(theta, ctheta, stheta, x, y);

  // R^T flip sign in sin
  trans2d.stheta = -1.0 * stheta;

  trans2d.theta = std::atan2(trans2d.stheta, trans2d.ctheta);
  // trans2d.ctheta = std::cos(trans2d.theta);

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
  twist_new.vy = twist.vy * ctheta + twist.vx * stheta - twist.w * x;


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
  x = ctheta * rhs.x - stheta * rhs.y + x;
  y = stheta * rhs.x + ctheta * rhs.y + y;
  theta += rhs.theta;
  ctheta = std::cos(theta);
  stheta = std::sin(theta);

  return *this;
}


TransformData2D Transform2D::displacement() const
{
  TransformData2D t2d;
  t2d.theta = this->theta;
  t2d.x = this->x;
  t2d.y = this->y;

  return t2d;
}



Transform2D Transform2D::integrateTwist(const Twist2D &twist) const
{
  // define a screw
  Screw2D S;

  // in Modern Robotics this is theta
  // for 1 unit of time beta = beta_dot
  auto beta = 0.0;

  // compose screw
  if (!almost_equal(twist.w, 0.0))
  {
    // beta is the magnitude of the angular twist velocity
    beta = std::abs(twist.w);

    S.w = twist.w / beta;
    S.vx = twist.vx / beta;
    S.vy = twist.vy / beta;
  }

  // all elements of twist are zero
  else if (almost_equal(twist.w, 0.0) and almost_equal(twist.vx, 0.0)
              and almost_equal(twist.vy, 0.0))
  {
    return *this;
  }

  else
  {
    // beta is the magnitude of the linear twist velocity
    beta = std::sqrt(std::pow(twist.vx, 2) + std::pow(twist.vy, 2));

    // the screw angular component is alread 0
    S.vx = twist.vx / beta;
    S.vy = twist.vy / beta;
  }


  // trig components of beta
  const auto cbeta = std::cos(beta);
  const auto sbeta = std::sin(beta);


  // compose new transform
  // rotation component
  const auto theta_new = std::atan2(sbeta * S.w, 1 + (1 - cbeta)*(-1.0 * std::pow(S.w,2)));

  const auto ctheta_new = std::cos(theta);
  const auto stheta_new = std::sin(theta);

  // translation component
  const auto x_new = S.vx*(beta + (beta - sbeta)*(-1.0 * std::pow(S.w,2))) + \
                S.vy*((1 - cbeta)*(-1.0 * S.w));


  const auto y_new = S.vx*((1 - cbeta) * S.w) + \
                    S.vy*(beta + (beta - sbeta)*(-1.0 * std::pow(S.w,2)));



  Transform2D T_new(theta_new, ctheta_new, stheta_new, x_new, y_new);
  Transform2D T(theta, ctheta, stheta, x, y);

  return T*T_new;
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
  return lhs *= rhs;
}



} // end namespace












// end file
