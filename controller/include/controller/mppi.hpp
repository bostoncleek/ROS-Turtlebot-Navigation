#ifndef MPPI_HPP
#define MPPI_HPP
/// \file
/// \brief Model predictive path integral control

#include <iosfwd>
#include <vector>
#include <eigen3/Eigen/Dense>

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
    void kinematicCart(const Ref<VectorXd> x_t,
                       const Ref<VectorXd> u_t,
                       Ref<VectorXd> x_dot)
    {
        x_dot(0) = (wheel_radius/2.0) * (u_t(0) + u_t(1)) * std::cos(x_t(2));
        x_dot(1) = (wheel_radius/2.0) * (u_t(0) + u_t(1)) * std::sin(x_t(2));
        x_dot(2) = (wheel_radius/wheel_base) * (u_t(1) - u_t(0));
    }


    double wheel_radius;
    double wheel_base;
  };


  /// \brief Loss functions
  struct LossFunc
  {
    /// \brief Set loss parameters using LQR cost
    /// \param Q - diagonal of square matrix to penalize error in states
    /// \param R - diagonal of square matrix to penalize controls
    /// \param P1 - diagonal of square matrix to penalize terminal error in states
    LossFunc(std::vector<double> Qdiag,
         std::vector<double> Rdiag,
         std::vector<double> P1diag)
    {
      Q = MatrixXd::Zero(3,3);
      Q(0,0) = Qdiag.at(0);
      Q(1,1) = Qdiag.at(1);
      Q(2,2) = Qdiag.at(2);

      R = MatrixXd::Zero(2,2);
      R(0,0) = Rdiag.at(0);
      R(1,1) = Rdiag.at(1);

      P1 = MatrixXd::Zero(3,3);
      P1(0,0) = P1diag.at(0);
      P1(1,1) = P1diag.at(1);
      P1(2,2) = P1diag.at(2);
    }

    /// \brief Compose loss using LQR cost
    /// \param x_t - current state (x,y,theta)
    /// \param x_d - desires state (xd,yd,thetad)
    /// \param u_t - controls (uL, uR)
    /// \return loss
    double loss(const Ref<VectorXd> x_t,
                const Ref<VectorXd> x_d,
                const Ref<VectorXd> u_t)
    {
      const VectorXd x_error = x_t - x_d;
      return (x_error.transpose()*Q*x_error)(0) + (u_t.transpose() * R * u_t)(0);
    }


    // /// \brief Compose loss using LQR cost
    // /// \param x_t - current state (x,y,theta)
    // /// \param x_T - terminal desires state (xd,yd,thetad)
    // /// \return terminal loss
    double terminalLoss(const Ref<VectorXd> x_t,
                        const Ref<VectorXd> x_T)
    {
      const VectorXd x_error = x_t - x_T;
      return (x_error.transpose()*P1*x_error)(0);
    }


    MatrixXd Q;
    MatrixXd R;
    MatrixXd P1;
  };





  /// \brief model predictive path integral control
  class MPPI
  {
  public:
    /// \brief contoller for diff drive robot
    /// \param cart_model - model of robot
    /// \param loss_func - loss function
    /// \param ul_var - sampling variance for left wheel velocity
    /// \param ur_var - sampling variance for right wheel velocity
    /// \param horizon - time horizon
    /// \param dt - time step for integrating model
    /// \param rollouts - number of rollouts
    MPPI(const CartModel &cart_model,
         const LossFunc &loss_func,
         double max_wheel_vel,
         double ul_var,
         double ur_var,
         double horizon,
         double dt,
         int rollouts);



  private:
    /// \brief Initialize kinematic cart model for RK4 simulation
    void initModel();

    RK4 rk4;
    CartModel cart_model;
    LossFunc loss_func;
    double max_wheel_vel, ul_var, ur_var;
    double horizon, dt;
    int rollouts;
  };
}














#endif
