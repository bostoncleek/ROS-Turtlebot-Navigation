// \file
/// \brief

#include <iostream>


#include "nuslam/filter.hpp"

namespace nuslam
{


std::mt19937_64 &getTwister()
{
  static std::random_device rd;
  static std::mt19937_64 gen(rd());
  return gen;
}


VectorXd sampleStandardNormal(int n)
{
  VectorXd rand_vec = Eigen::VectorXd::Zero(n);
  for(auto i = 0; i < n; i++)
  {
    std::normal_distribution<double> dis(0, 1);
    rand_vec(i) = dis(getTwister());
  }
  return rand_vec;
}


VectorXd sampleMultivariateDistribution(MatrixXd &cov)
{
  int dim = cov.cols();
  VectorXd rand_vec = sampleStandardNormal(dim);

  // cholesky decomposition
  Eigen::MatrixXd L( cov.llt().matrixL() );

  return L * rand_vec;
}




EKF::EKF()
{
  // init motion noise sampling
  motion_noise = Eigen::MatrixXd::Zero(3,3);
  motion_noise(0) = 1e-10;  // theta var
  motion_noise(1) = 1e-5;   // x var
  motion_noise(2) = 1e-5;   // y var

}


void EKF::initState(Pose pose, std::vector<Vector2D> lm)
{
  int num_states = 3 + 2 * lm.size();
  MatrixXd st = Eigen::MatrixXd::Zero(num_states, 1);

  st(0) = pose.theta;
  st(1) = pose.x;
  st(2) = pose.y;

  // std::cout << "lm size: " << lm.size() << std::endl;

  auto i = 0;
  for(const auto &m : lm)
  {
    int lm_x = 3 + i;
    int lm_y = 3 + i + 1;

    st(lm_x) = m.x;
    st(lm_y) = m.y;

    i += 2;
  }

  state = st;

  // std::cout << state << std::endl;
}



void EKF::initCov(int num_lm)
{
  int num_rows = 3 + 2 * num_lm;
  int num_cols = 3 + 2 * num_lm;

  // set pose to (0,0,0)
  MatrixXd cov = Eigen::MatrixXd::Zero(num_rows, num_cols);

  // set landmarks to a large number
  for(auto i = 0; i < 2*num_lm; i++)
  {
    auto row = 3 + i;
    auto col = 3 + i;

    cov(row, col) = 1e12;
  }

  state_cov = cov;

  // std::cout << state_cov << std::endl;
}



void EKF::motionUpdate(Twist2D u)
{
  // sample noise
  VectorXd w = sampleMultivariateDistribution(motion_noise);


  // update robot pose based on odometry
  if (rigid2d::almost_equal(u.w, 0.0))
  {
    // update theta
    state(0) += w(0);
    // update x
    state(1) += u.vx * std::cos(state(2)) + w(1);
    // update y
    state(2) += u.vx * std::sin(state(2)) + w(2);
  }

  else
  {
    // update theta
    state(0) += u.w + w(0);
    // update x
    state(1) += (-u.vx / u.w) * std::sin(state(0)) + \
                            (u.vx / u.w) * std::sin(state(0) + u.w) + w(1);
    // update y
    state(2) += (u.vx / u.w) * std::cos(state(0)) - \
                            (u.vx / u.w) * std::cos(state(0) + u.w) + w(2);
  }

  std::cout << state << std::endl;
}












} // end namespace
