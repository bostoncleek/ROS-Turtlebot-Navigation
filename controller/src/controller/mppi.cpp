/// \file
/// \brief Model predictive path integral control

#include <iostream>
#include <functional>
#include "controller/mppi.hpp"


namespace controller
{
MPPI::MPPI(const CartModel &cart_model,
         double max_wheel_vel,
         double ul_var,
         double ur_var,
         double horizon,
         double dt,
         int rollouts)
         : rk4(dt),
           max_wheel_vel(max_wheel_vel),
           ul_var(ul_var),
           ur_var(ur_var),
           horizon(horizon),
           dt(dt),
           rollouts(rollouts)
{

}


}
