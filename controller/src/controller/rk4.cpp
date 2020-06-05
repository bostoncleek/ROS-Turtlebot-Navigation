/// \file
/// \brief 4th order Runge Kutta method

#include <iostream>
#include "controller/rk4.hpp"

namespace controller
{

RK4::RK4(double step) : step(step)
{
}


void RK4::registerODE(std::function<void(const Ref<VectorXd>, Ref<VectorXd>)> ode_func)
{
  func = ode_func;
}


void RK4::registerODE(std::function<void(const Ref<VectorXd>, const Ref<VectorXd>, Ref<VectorXd>)> ode_func)
{
  func_cntrl = ode_func;
}


MatrixXd RK4::solve(const Ref<VectorXd> x0, double horizon)
{
  if(!func)
  {
    std::cerr << "Function not registered!" << std::endl;
    exit(EXIT_FAILURE);
  }

  VectorXd state = x0;
  const auto num_samples = static_cast<int>(horizon/step);
  MatrixXd trajectory(state.rows(), num_samples);

  for(int i = 0; i < num_samples; i++)
  {
    integrate(state);
    trajectory.col(i) = state;
  }

  return trajectory;
}


MatrixXd RK4::solve(const Ref<VectorXd> x0, const Ref<MatrixXd> u, double horizon)
{
  if(!func_cntrl)
  {
    std::cerr << "Function not registered!" << std::endl;
    exit(EXIT_FAILURE);
  }

  VectorXd state = x0;
  const auto num_samples = static_cast<int>(horizon/step);
  MatrixXd trajectory(state.rows(), num_samples);

  for(int i = 0; i < num_samples; i++)
  {
    VectorXd u_t = u.col(i);
    integrate(state, u_t);
    trajectory.col(i) = state;
  }

  return trajectory;
}


void RK4::integrate(Ref<VectorXd> x_t)
{
  // use arg as a temp vector so the input can be passed
  // in using a reference
  VectorXd arg = VectorXd::Zero(x_t.size());
  VectorXd k1, k2, k3, k4;
  k1 = k2 = k3 = k4 = arg;

  func(x_t, k1);

  arg = x_t + step*(0.5*k1);
  func(arg, k2);

  arg = x_t + step*(0.5*k2);
  func(arg, k3);

  arg = x_t + step*k3;
  func(arg, k4);

  x_t = x_t + (step/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);
}


void RK4::integrate(Ref<VectorXd> x_t, const Ref<VectorXd> u_t)
{
  // use arg as a temp vector so the input can be passed
  // in using a reference
  VectorXd arg = VectorXd::Zero(x_t.size());
  VectorXd k1, k2, k3, k4;
  k1 = k2 = k3 = k4 = arg;

  func_cntrl(x_t, u_t, k1);

  arg = x_t + step*(0.5*k1);
  func_cntrl(arg, u_t, k2);

  arg = x_t + step*(0.5*k2);
  func_cntrl(arg, u_t, k3);

  arg = x_t + step*k3;
  func_cntrl(arg, u_t, k4);

  x_t = x_t + (step/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);
}




}
