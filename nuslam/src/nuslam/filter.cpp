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
  VectorXd rand_vec = VectorXd::Zero(n);
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




EKF::EKF(int num_lm) : n(num_lm)
{
  // init motion noise
  motion_noise = MatrixXd::Zero(3,3);
  motion_noise(0,0) = 1e-10;  // theta var
  motion_noise(1,1) = 1e-5;   // x var
  motion_noise(2,2) = 1e-5;   // y var
  // std::cout << motion_noise << std::endl;

  // init state and covariance
  initState();
  initCov();

  // init process noise
  initProcessNoise();
}


void EKF::initState()
{
  auto num_states = 3 + 2 * n;
  state = VectorXd::Zero(num_states);
  // std::cout << state << std::endl;
}



void EKF::initCov()
{
  auto num_rows = 3 + 2 * n;
  auto num_cols = 3 + 2 * n;
  state_cov = MatrixXd::Zero(num_rows, num_cols);

  // set pose to (0,0,0)

  // set landmarks to a large number
  for(auto i = 0; i < 2*n; i++)
  {
    auto row = 3 + i;
    auto col = 3 + i;
    state_cov(row, col) = 1e12;
  }

  // std::cout << state_cov << std::endl;
}


void EKF::initProcessNoise()
{
  auto num_rows = 3 + 2 * n;
  auto num_cols = 3 + 2 * n;
  process_noise = MatrixXd::Zero(num_rows, num_cols);

  process_noise(0,0) = motion_noise(0,0);
  process_noise(1,1) = motion_noise(1,1);
  process_noise(2,2) = motion_noise(2,2);

  // std::cout << process_noise << std::endl;
}


void EKF::motionUpdate(const Twist2D &u)
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

  // std::cout << state << std::endl;
}


void EKF::uncertaintyUpdate(const Twist2D &u, Ref<MatrixXd> sigma_bar)
{
  auto num_rows = 3 + 2 * n;
  auto num_cols = 3 + 2 * n;

  MatrixXd I = MatrixXd::Identity(num_rows, num_cols);
  // jocobian of motion model
  MatrixXd G = MatrixXd::Zero(num_rows, num_cols);


  if (rigid2d::almost_equal(u.w, 0.0))
  {
    G(1,0) = -u.vx * std::sin(state(0));
    G(2,0) = u.vx * std::cos(state(0));
  }

  else
  {
    G(1,0) = (-u.vx / u.w) * std::cos(state(0)) + \
                            (u.vx / u.w) * std::cos(state(0) + u.w);

    G(2,0) = (-u.vx / u.w) * std::sin(state(0)) + \
                            (u.vx / u.w) * std::sin(state(0) + u.w);
  }

  // add I to G
  G += I;
  // std::cout << G << std::endl;

  // predicted covariance
  sigma_bar = G * state_cov * G.transpose() + process_noise;
  // std::cout << sigma_bar << std::endl;
}








} // end namespace


















// void EKF::initCov(int num_lm)
// {
//   int num_rows = 3 + 2 * num_lm;
//   int num_cols = 3 + 2 * num_lm;
//
//   // set pose to (0,0,0)
//   MatrixXd cov = Eigen::MatrixXd::Zero(num_rows, num_cols);
//
//   // set landmarks to a large number
//   for(auto i = 0; i < 2*num_lm; i++)
//   {
//     auto row = 3 + i;
//     auto col = 3 + i;
//
//     cov(row, col) = 1e12;
//   }
//
//   state_cov = cov;
//
//   // std::cout << state_cov << std::endl;
// }



// void EKF::initState(const Pose &pose, const std::vector<Vector2D> &lm)
// {
//   int num_states = 3 + 2 * lm.size();
//   MatrixXd st = Eigen::MatrixXd::Zero(num_states, 1);
//
//   st(0) = pose.theta;
//   st(1) = pose.x;
//   st(2) = pose.y;
//
//   // std::cout << "lm size: " << lm.size() << std::endl;
//
//   auto i = 0;
//   for(const auto &m : lm)
//   {
//     int lm_x = 3 + i;
//     int lm_y = 3 + i + 1;
//
//     st(lm_x) = m.x;
//     st(lm_y) = m.y;
//
//     i += 2;
//   }
//
//   state = st;
//
//   // std::cout << state << std::endl;
// }
