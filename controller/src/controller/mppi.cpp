/// \file
/// \brief Model predictive path integral control for turtlebot3
///        waypoint following

#include <iostream>
#include <algorithm>
#include <functional>
#include "controller/mppi.hpp"
#include "rigid2d/utilities.hpp"


namespace controller
{

void cumSumCost(const Ref<MatrixXd> input, Ref<MatrixXd> result)
{
  const auto rows = input.rows();
  // const auto cols = input.cols();
  // last row
  result.row(rows-1) = input.row(rows-1);
  for(int i = rows-2; i >=0; i--)
  {
    result.row(i) = input.row(i) + result.row(i+1);
  }
}


MPPI::MPPI(const CartModel &cart_model,
           const LossFunc &loss_func,
           double lambda,
           double max_wheel_vel,
           double ul_var,
           double ur_var,
           double horizon,
           double dt,
           int rollouts)
             : rk4(dt),
               cart_model(cart_model),
               loss_func(loss_func),
               lambda(lambda),
               max_wheel_vel(max_wheel_vel),
               ul_var(ul_var),
               ur_var(ur_var),
               horizon(horizon),
               dt(dt),
               rollouts(rollouts),
               steps(static_cast<int>(horizon/dt))
{
  initModel();
  initController();
}


void MPPI::setInitialControls(double uL, double uR)
{
  uinit(0) = uL;
  uinit(1) = uR;

  u.row(0) = MatrixXd::Constant(1, steps, uinit(0));
  u.row(1) = MatrixXd::Constant(1, steps, uinit(1));
}


void MPPI::setWaypoint(const Pose &wpt)
{
  xd(0) = wpt.x;
  xd(1) = wpt.y;
  xd(2) = wpt.theta;
}


WheelVelocities MPPI::newControls(const Pose &ps)
{
  // I.C.
  VectorXd x0(3);
  x0 << ps.x, ps.y, ps.theta;

  // loss matrix constains loss at each point for each rollout
  MatrixXd loss_mat = MatrixXd::Zero(steps,rollouts);

  for(int k = 0; k < rollouts; k++)
  {
    // perturb constrols
    MatrixXd pert = MatrixXd::Zero(2,steps);
    pertubations(pert);

    // store pertubations for control update
    duL.col(k) = pert.row(0).transpose();
    duR.col(k) = pert.row(1).transpose();


    // controls used to simulate
    MatrixXd u_pert = u + pert;

    // simulate vehicle
    MatrixXd traj = rk4.solve(x0, u_pert, horizon);

    // compose loss
    for(int i = 0; i < steps; i++)
    {
      loss_mat(i,k) = loss_func.loss(traj.col(i), xd, u_pert.col(i));
    }

    // terminal loss
    loss_mat(steps-1,k) = loss_func.terminalLoss(traj.col(steps-1), xd);
  } // end rollouts

  // use a reduction to compose the cost
  cumSumCost(loss_mat, J);


  for(int i = 0; i < steps; i++)
  {
    // subtract min cost across each rollout
    J.array().row(i) -=  J.row(i).minCoeff();

    VectorXd w = Eigen::exp((J.array().row(i) * -1.0/lambda)) + 1e-8;
    w = w.array() * (1.0 / w.sum());

    u(0,i) += w.dot(duL.row(i));
    u(1,i) += w.dot(duR.row(i));

    // saturate controls
    u(0,i) = std::clamp(u(0,i), -max_wheel_vel, max_wheel_vel);
    u(1,i) = std::clamp(u(1,i), -max_wheel_vel, max_wheel_vel);
  }

  // send first column controls to robot
  WheelVelocities wheel_vel;
  wheel_vel.ul = u(0,0);
  wheel_vel.ur = u(1,0);

  // shift controls over for next iteration
  u.leftCols(u.cols()-1) = u.rightCols(u.cols()-1);
  // re-initialize the last control
  u(0,u.cols()-1) = uinit(0);
  u(1,u.cols()-1) = uinit(1);

  return wheel_vel;
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


void MPPI::initController()
{
  // init control signal
  uinit = VectorXd::Zero(2);
  u = MatrixXd::Zero(2,steps);

  // set cost matrix and stored perturbations to zero
  J = MatrixXd::Zero(steps,rollouts);
  duL = MatrixXd::Zero(steps,rollouts);
  duR = MatrixXd::Zero(steps,rollouts);

  // waypoint goal
  xd = VectorXd::Zero(3);
}


void MPPI::pertubations(Ref<MatrixXd> pert)
{
  // random generator uses standard deviation
  const auto ul_sig = std::sqrt(ul_var);
  const auto ur_sig = std::sqrt(ur_var);

  for(int i = 0; i < steps; i++)
  {
    pert(0,i) = rigid2d::sampleNormalDistribution(0.0, ul_sig);
    pert(1,i) = rigid2d::sampleNormalDistribution(0.0, ur_sig);
  }
}

}
