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
  // must be square
  int dim = cov.cols();
  VectorXd rand_vec = sampleStandardNormal(dim);

  // cholesky decomposition
  Eigen::MatrixXd L( cov.llt().matrixL() );

  return L * rand_vec;
}





EKF::EKF(int num_lm) : n(num_lm)
{
  state_size = 3 + 2 * n;

  // init motion noise
  motion_noise = MatrixXd::Zero(3,3);
  motion_noise(0,0) = 1e-10;  // theta var
  motion_noise(1,1) = 1e-5;   // x var
  motion_noise(2,2) = 1e-5;   // y var
  // std::cout << motion_noise << std::endl;

  // init measurement noise
  measurement_noise = MatrixXd::Zero(2,2);
  measurement_noise(0,0) = 1e-5;   // r var
  measurement_noise(1,1) = 1e-5;   // b var
  // std::cout << measurement_noise << std::endl;

  // init state and covariance
  initState();
  initCov();

  // init process noise
  initProcessNoise();
}


void EKF::initState()
{
  state = VectorXd::Zero(state_size);
  // std::cout << state << std::endl;
}



void EKF::initCov()
{
  state_cov = MatrixXd::Zero(state_size, state_size);

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
  process_noise = MatrixXd::Zero(state_size, state_size);

  process_noise(0,0) = motion_noise(0,0);
  process_noise(1,1) = motion_noise(1,1);
  process_noise(2,2) = motion_noise(2,2);

  // std::cout << process_noise << std::endl;
}


void EKF::setKnownLandamrks(const std::vector<Vector3d> &landmarks)
{
  lm = landmarks;
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
  MatrixXd I = MatrixXd::Identity(state_size, state_size);
  // jocobian of motion model
  MatrixXd G = MatrixXd::Zero(state_size, state_size);


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



void EKF::measurementJacobian(double dx, double dy, int j, Ref<MatrixXd> H)
{
  const auto q = dx*dx + dy*dy;
  const auto sqrt_q = std::sqrt(q);

  MatrixXd F = MatrixXd::Zero(5, state_size);
  // MatrixXd H = MatrixXd::Zero(2, num_cols);
  MatrixXd h = MatrixXd::Zero(2, 5);

  // upper left is diagonal for pose
  F(0,0) = 1;
  F(1,1) = 1;
  F(2,2) = 1;

  // lower right diagonal
  F(3, 2 * j + 3) = 1;
  F(4, 2 * j + 4) = 1;
  // std::cout << F << std::endl;


  // // row 1
  h(0,1) = (-sqrt_q * dx) / q;
  h(0,2) = (-sqrt_q * dy) / q;
  h(0,3) = (sqrt_q * dx) / q;
  h(0,4) = (sqrt_q * dy) / q;

  // // row 2
  h(1,0) = -1.0;
  h(1,1) = dy / q;
  h(1,2) = -dx / q;
  h(1,3) = -dy /q;
  h(1,4) = dx / q;


  H = h * F;
  // std::cout << H << std::endl;
}


Vector2d EKF::predictedMeasurement(int j)
{
  // index of jth correspondence
  // first 3 indecis are for (theta, x, y)
  const auto jx = 3 + j;
  const auto jy = 3 + j + 1;

  // change in x landmark to robot
  const auto delta_x = state(jx) - state(1);
  const auto delta_y = state(jy) - state(2);

  // measurement noise
  VectorXd v = sampleMultivariateDistribution(measurement_noise);

  Vector2d z_hat;
  z_hat(0) = std::sqrt(delta_x * delta_x + delta_y * delta_y) + v(0);
  z_hat(1) = std::atan2(delta_y, delta_x) - state(0) + v(1);
  // std::cout << z_hat << std::endl;

  return z_hat;
}


void EKF::knownCorrespondenceSLAM(const std::vector<Vector2D> &meas, const Twist2D &u)
{
  // 1) motion model
  motionUpdate(u);

  // 2) propagate uncertainty
  Eigen::MatrixXd sigma_bar = Eigen::MatrixXd::Zero(state_size, state_size);
  uncertaintyUpdate(u, sigma_bar);

  // 3) update state based on observations 
  // convert landmark positions in robot frame to map frame
  std::vector<Vector2D> lm_meas = meas;
  measRobotToMap(lm_meas);



  // check if j = -1 for lm correspondence















}



int EKF::findKnownCorrespondence(const Vector2D m)
{
  auto smallest_d = 1e12;
  auto index = -1;


  // compute distance to every landmark
  for(unsigned int i = 0; i < lm.size(); i++)
  {
    // known landmark
    Vector2D know_m(lm.at(i)(0), lm.at(i)(1));
    // distance from known landmark to measurement (m)
    const auto d = pointDistance(know_m, m);

    // update if smaller
    if (d < smallest_d)
    {
      smallest_d = d;
      index = i;
    }
  }

  return index;
}


void EKF::measRobotToMap(std::vector<Vector2D> &meas)
{
  for(auto &m : meas)
  {
    m.x += state(1);
    m.y += state(2);
  }
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
