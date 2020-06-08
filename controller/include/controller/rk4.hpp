#ifndef RK4_HPP
#define RK4_HPP
/// \file
/// \brief 4th order Runge Kutta method

#include <functional>
#include <eigen3/Eigen/Dense>
#include <iosfwd>

namespace controller
{
  using Eigen::MatrixXd;
  using Eigen::VectorXd;
  using Eigen::Vector2d;
  using Eigen::Ref;


  /// \brief Integrator
  class RK4
  {
    public:
      /// \brief Integrator with fixed step size
      /// \param step - time step
      RK4(double step);

      /// \brief Register an autonomous ODE
      /// \param ode_func - function to integrate
      void registerODE(std::function<void(const Ref<VectorXd>, Ref<VectorXd>)> ode_func);

      /// \brief Register an autonomous ODE
      /// \param ode_func - function to integrate
      void registerODE(std::function<void(const Ref<VectorXd>, const Ref<VectorXd>, Ref<VectorXd>)> ode_func);

      /// \brief Solve ODE or system of ODEs
      /// \param x0 - initial condition
      /// \param horizon - final time of integration
      MatrixXd solve(const Ref<VectorXd> x0, double horizon);

      /// \brief Solve ODE or system of ODEs
      /// \param x0 - initial condition
      /// \param u - control signal (must be of length equal to number of rk4 iterations => horizon/step)
      /// \param horizon - final time of integration
      MatrixXd solve(const Ref<VectorXd> x0, const Ref<MatrixXd> u, double horizon);
    private:
      /// \brief Perform one iteration of rk4
      /// x_t[out] - update the current state x_t
      void integrate(Ref<VectorXd> x_t);

      /// \brief Perform one iteration of rk4
      /// param u_t - control vector
      /// x_t[out] - update the current state x_t
      void integrate(Ref<VectorXd> x_t, const Ref<VectorXd> u_t);

      // ODE args: x(t), u(t), xdot[out]
      std::function<void(const Ref<VectorXd>, const Ref<VectorXd>, Ref<VectorXd>)> func_cntrl;

      // ODE args: x(t), xdot[out]
      std::function<void(const Ref<VectorXd>, Ref<VectorXd>)> func;
      double step;                                                            // time step
  };
}
#endif
