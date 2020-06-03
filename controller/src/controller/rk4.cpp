/// \file
/// \brief 4th order Runge Kutta method

#include "controller/rk4.hpp"

namespace controller
{

RK4::RK4(double dt) : dt(dt)
{
}


// // Solve Atunomous Diff Eq
// void RK4::RegisterODE(std::function<Eigen::VectorXd(const Eigen::VectorXd)> ode_func) {
//   ode_auto_func_ = ode_func;
// }
//
//
// Eigen::MatrixXd RK4::Solve(Eigen::VectorXd& curr_state, double horizon) {
//   if(!ode_auto_func_) {
//     std::cerr << "ODE function not registered!" << std::endl;
//     exit(EXIT_FAILURE);
//   }
//   int num_samples = (int)(horizon/int_step_);
//   Eigen::MatrixXd trajectory(curr_state.rows(),num_samples);
//   for(int ii=0; ii<num_samples; ii++) {
//     RK4Iteration(curr_state);
//     trajectory.col(ii) = curr_state;
//   }
//   return trajectory;
// }
//
// void RK4::RK4Iteration(Eigen::VectorXd& x_t) {
//   Eigen::VectorXd k1,k2,k3,k4;
//   k1 = ode_auto_func_(x_t);
//   k2 = ode_auto_func_(x_t+int_step_*(0.5*k1));
//   k3 = ode_auto_func_(x_t+int_step_*(0.5*k2));
//   k4 = ode_auto_func_(x_t+int_step_*k3);
//   x_t = x_t+(int_step_/6.0)*(k1+2*k2+2*k3+k4);
// }
//
// // Solve Diff Eq with system inputs
// void RK4::RegisterODE(std::function<Eigen::VectorXd(const Eigen::VectorXd,const Eigen::VectorXd)> ode_func) {
//   ode_func_ = ode_func;
// }
//
// Eigen::MatrixXd RK4::Solve(Eigen::VectorXd& curr_state, const Eigen::VectorXd& u_t, double horizon) {
//   if(!ode_func_) {
//     std::cerr << "ODE function not registered!" << std::endl;
//     exit(EXIT_FAILURE);
//   }
//   int num_samples = (int)(horizon/int_step_);
//   Eigen::MatrixXd trajectory(curr_state.rows(),num_samples);
//   for(int ii=0; ii<num_samples; ii++) {
//     RK4Iteration(curr_state,u_t);
//     trajectory.col(ii) = curr_state;
//   }
//   return trajectory;
// }
//
// void RK4::RK4Iteration(Eigen::VectorXd& x_t, const Eigen::VectorXd& u_t) {
//   Eigen::VectorXd k1,k2,k3,k4;
//   k1 = ode_func_(x_t,u_t);
//   k2 = ode_func_(x_t+int_step_*(0.5*k1),u_t);
//   k3 = ode_func_(x_t+int_step_*(0.5*k2),u_t);
//   k4 = ode_func_(x_t+int_step_*k3,u_t);
//   x_t = x_t+(int_step_/6.0)*(k1+2*k2+2*k3+k4);
// }

}
