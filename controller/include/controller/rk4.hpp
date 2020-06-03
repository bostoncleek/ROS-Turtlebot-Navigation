#ifndef RK4_HPP
#define RK4_HPP
/// \file
/// \brief 4th order Runge Kutta method

#include <functional>
#include <eigen3/Eigen/Dense>
#include <iosfwd>

namespace controller
{
  /// \brief Integrator
  class RK4 {
    public:
      /// \brief Integrator with fixed step size
      /// \param dt - time step 
      RK4(double dt);

      // void RegisterODE(std::function<Eigen::VectorXd(const Eigen::VectorXd,const Eigen::VectorXd)> ode_func);
      //
      // void RegisterODE(std::function<Eigen::VectorXd(const Eigen::VectorXd)> ode_func);
      //
      // Eigen::MatrixXd Solve(Eigen::VectorXd& curr_state, double horizon);
      //
      // Eigen::MatrixXd Solve(Eigen::VectorXd& curr_state, const Eigen::VectorXd& u_t, double horizon);
    private:
      // void RK4Iteration(Eigen::VectorXd& x_t,const Eigen::VectorXd& u_t);
      //
      // void RK4Iteration(Eigen::VectorXd& x_t);
      //
      // std::function<Eigen::VectorXd(const Eigen::VectorXd)> ode_auto_func_;
      //
      // std::function<Eigen::VectorXd(const Eigen::VectorXd,const Eigen::VectorXd)> ode_func_;

      double dt;
  };



}

#endif
