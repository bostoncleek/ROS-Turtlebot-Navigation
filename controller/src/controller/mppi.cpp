/// \file
/// \brief Model predictive path integral control

#include <iostream>
#include <functional>
#include "controller/mppi.hpp"


namespace controller
{
MPPI::MPPI(const CartModel &cart_model,
           const LossFunc &loss_func,
           double max_wheel_vel,
           double ul_var,
           double ur_var,
           double horizon,
           double dt,
           int rollouts)
             : rk4(dt),
               cart_model(cart_model),
               loss_func(loss_func),
               max_wheel_vel(max_wheel_vel),
               ul_var(ul_var),
               ur_var(ur_var),
               horizon(horizon),
               dt(dt),
               rollouts(rollouts)
{
  initModel();
}


void MPPI::initModel()
{
  std::function<void(const Ref<VectorXd>,
                     const Ref<VectorXd>,
                     Ref<VectorXd>)> func_cntrl = std::bind(&CartModel::kinematicCart,
                                                            &cart_model,
                                                            std::placeholders::_1,
                                                            std::placeholders::_2,
                                                            std::placeholders::_3);

  rk4.registerODE(func_cntrl);
}











}
