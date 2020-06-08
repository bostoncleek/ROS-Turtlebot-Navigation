#ifndef MPPI_HPP
#define MPPI_HPP
/// \file
/// \brief Model predictive path integral control

#include <eigen3/Eigen/Dense>
#include <iosfwd>

#include <rigid2d/rigid2d.hpp>
#include <rigid2d/diff_drive.hpp>
#include "controller/rk4.hpp"

namespace controller
{
  using Eigen::MatrixXd;
  using Eigen::VectorXd;
  using Eigen::Ref;

  using rigid2d::Pose;
  using rigid2d::Vector2D;


  /// \brief ODEs for kinematic model of turtlebot
  struct CartModel
  {
    CartModel(double wheel_radius, double wheel_base)
                        : wheel_radius(wheel_radius),
                          wheel_base(wheel_base) {}

    /// \brief Kinematic model of turtlebot
    /// \param x_t - current state (x,y,theta)
    /// \param u_t - wheel velocities (uL, uR)
    /// xdot[out] - (dx/dt, dy/dt, dtheta/dt)
    void kinematicCart(const Eigen::Ref<Eigen::VectorXd> x_t,
                       const Eigen::Ref<Eigen::VectorXd> u_t,
                       Eigen::Ref<Eigen::VectorXd> x_dot)
    {
        x_dot(0) = (wheel_radius/2.0) * (u_t(0) + u_t(1)) * std::cos(x_t(2));
        x_dot(1) = (wheel_radius/2.0) * (u_t(0) + u_t(1)) * std::sin(x_t(2));
        x_dot(2) = (wheel_radius/wheel_base) * (u_t(1) - u_t(0));
    }


    double wheel_radius;
    double wheel_base;
  };



  /// \brief model predictive path integral control
  class MPPI
  {
  public:
    MPPI(int rollouts);


  private:

  };
}
#endif