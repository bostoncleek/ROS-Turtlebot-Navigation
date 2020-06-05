/// \file
/// \brief 4th order Runge Kutta method

#include <iostream>
#include "controller/rk4.hpp"

namespace controller
{

RK4::RK4(double step) : step(step)
{
}


void RK4::registerODE(std::function<VectorXd(Ref<VectorXd>)> ode_func)
{
  func = ode_func;
}


MatrixXd RK4::solve(const Ref<VectorXd> x0, double horizon)
{
  if(!func)
  {
    std::cerr << "function not registered!" << std::endl;
    exit(EXIT_FAILURE);
  }

  VectorXd state = x0;
  const auto num_samples = (int)(horizon/step);
  MatrixXd trajectory(state.rows(), num_samples);

  for(int i=0; i < num_samples; i++)
  {
    integrate(state);
    trajectory.col(i) = state;
  }

  return trajectory;
}


void RK4::integrate(Ref<VectorXd> x_t)
{
  VectorXd k1, k2, k3, k4;
  k1 = func(x_t);
  // k2 = step*func(x_t + 0.5*k1);
  // k2 = step*func(x_t + 0.5*k1);
  // k3 = step*func(x_t + 0.5*k2);
  // k4 = step*func(x_t + k3);
  // x_t = x_t + (1.0/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);
}


















}
