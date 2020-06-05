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
  using Eigen::Ref;


  /// \brief Integrator
  class RK4 {
    public:
      /// \brief Integrator with fixed step size
      /// \param step - time step
      RK4(double dt);

      /// \brief Register an autonomous ODE or system of ODEs
      /// \param ode_func - function or system to integrate
      void registerODE(std::function<VectorXd(Ref<VectorXd>)> ode_func);

      /// \brief Solve ODE or system of ODEs
      /// \param
      /// \param
      MatrixXd solve(Ref<VectorXd> x0, double horizon);

    private:
      /// \brief Perform one iteration of rk4
      /// x_t[out] - update the current state x_t
      void integrate(Ref<VectorXd> x_t);

      std::function<VectorXd(Ref<VectorXd>)> func;      // function to integrate
      double step;                                      // time step
  };



}

#endif
