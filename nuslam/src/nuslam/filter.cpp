// \file
/// \brief

#include <iostream>
#include <algorithm>
#include <stdexcept>

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


VectorXd sampleMultivariateDistribution(const MatrixXd &cov)
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
  // total state vector size robot plus landmarks
  state_size = 3 + 2 * n;

  // distance threshold for the known correspondences case
  // for determining which landmark we are looking at
  known_dist_thresh = 0.1;

  // init motion noise
  motion_noise = MatrixXd::Zero(3,3);
  // motion_noise(0,0) = 1e-10;  // theta var
  // motion_noise(1,1) = 1e-5;   // x var
  // motion_noise(2,2) = 1e-5;   // y var
  // std::cout << motion_noise << std::endl;

  // init measurement noise
  measurement_noise = MatrixXd::Zero(2,2);
  // measurement_noise(0,0) = 1e-5;   // r var
  // measurement_noise(1,1) = 1e-5;   // b var
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
  if (almost_equal(u.w, 0.0))
  {
    // update theta
    state(0) = normalize_angle_PI(state(0) + w(0));
    // update x
    state(1) += u.vx * std::cos(state(2)) + w(1);
    // update y
    state(2) += u.vx * std::sin(state(2)) + w(2);
  }

  else
  {
    // update theta
    state(0) = normalize_angle_PI(state(0) + u.w + w(0));
    // update x
    state(1) += (-u.vx / u.w) * std::sin(state(0)) + \
                            (u.vx / u.w) * std::sin(state(0) + u.w) + w(1);
    // update y
    state(2) += (u.vx / u.w) * std::cos(state(0)) - \
                            (u.vx / u.w) * std::cos(state(0) + u.w) + w(2);
  }

  // std::cout << state << std::endl;
}


void EKF::uncertaintyUpdate(const Twist2D &u, Ref<MatrixXd> sigma_bar) const
{
  MatrixXd I = MatrixXd::Identity(state_size, state_size);
  // jocobian of motion model
  MatrixXd G = MatrixXd::Zero(state_size, state_size);


  if (almost_equal(u.w, 0.0))
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



void EKF::measurementJacobian(const int j, Ref<MatrixXd> H) const
{
  // difference between landmark and robot
  Vector2d lm_st = landmarkState(j);
  const auto dx = lm_st(0) - state(1);
  const auto dy = lm_st(1) - state(2);


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
  F(3, 2*j + 3) = 1;
  F(4, 2*j + 4) = 1;
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


Vector2d EKF::predictedMeasurement(const int j) const
{
  // index of jth correspondence
  // first 3 indecis are for (theta, x, y)
  const auto jx = 2*j + 3;
  const auto jy = 2*j + 4;

  // change in x landmark to robot
  const auto delta_x = state(jx) - state(1);
  const auto delta_y = state(jy) - state(2);

  // measurement noise
  VectorXd v = sampleMultivariateDistribution(measurement_noise);

  Vector2d z_hat;
  z_hat(0) = std::sqrt(delta_x * delta_x + delta_y * delta_y) + v(0);
  z_hat(1) = normalize_angle_PI(std::atan2(delta_y, delta_x) - normalize_angle_PI(state(0)) + v(1));
  // std::cout << z_hat << std::endl;

  return z_hat;
}


Vector2d EKF::landmarkState(const int j) const
{
  // index of jth correspondence
  // first 3 indecis are for (theta, x, y)
  const auto jx = 2*j + 3;
  const auto jy = 2*j + 4;

  Vector2d st;
  st << state(jx), state(jy);

  return st;
}


void EKF::newLandmark(const LM &m, const int j)
{
  // index of jth correspondence
  // first 3 indecis are for (theta, x, y)
  const auto jx = 2*j + 3;
  const auto jy = 2*j + 4;

  state(jx) = state(1) + m.r * std::cos(m.b + state(0));
  state(jy) = state(2) + m.r * std::sin(m.b + state(0));
}





void EKF::knownCorrespondenceSLAM(const std::vector<Vector2D> &meas, const Twist2D &u)
{
  // 1) motion model
  motionUpdate(u);


  // // 2) propagate uncertainty
  Eigen::MatrixXd sigma_bar = MatrixXd::Zero(state_size, state_size);
  uncertaintyUpdate(u, sigma_bar);


  // 3) update state based on observations
  // measurements come in as (x,y) in robot frame
  // convert them to range and bearing
  // convert landmark positions in robot frame to map frame
  std::vector<LM> lm_meas(meas.size());
  measRobotToMap(meas, lm_meas);


  for(const auto m : lm_meas)
  {
    // find correspondence based in id
    int j = findKnownCorrespondence(m);
    // std::cout << j << std::endl;


    // not found in list
    // this is a problem
    if (j == -1)
    {
      throw std::invalid_argument("Known landmark not found in correspondence list!");
    }

    // landmark has not been scene before
    if (std::find(lm_j.begin(), lm_j.end(), j) == lm_j.end())
    {
      // std::cout << "new landmark id: " << j << std::endl;
      // add id to observed list
      // init new landmark with measurement
      lm_j.push_back(j);
      newLandmark(m, j);
    }


    // 4) predicted measurement
    Vector2d z_hat = predictedMeasurement(j);
    // std::cout << z_hat << std::endl;


    // 5) measurement jacobian
    MatrixXd H = MatrixXd::Zero(2, state_size);
    measurementJacobian(j, H);


    // 6) kalman gain
    // MatrixXd temp = MatrixXd::Zero(2,2);
    MatrixXd temp = H * sigma_bar * H.transpose() + measurement_noise;
    MatrixXd temp_inv = temp.inverse();

    MatrixXd K = sigma_bar * H.transpose() * temp_inv;
    // std::cout << "kalman Gain" << std::endl;
    // std::cout << K << std::endl;


    // 7) Update the state vector
    // difference in measurements
    Vector2d delta_z;
    delta_z(0) = m.r - z_hat(0);
    delta_z(1) = normalize_angle_PI(normalize_angle_PI(m.b) - normalize_angle_PI(z_hat(0)));

    state += K * delta_z;
    std::cout << "State" << std::endl;
    std::cout << state << std::endl;


    // 8) Update covariance
    // state_cov +=


  } // end loop





}



int EKF::findKnownCorrespondence(const LM &m) const
{
  auto smallest_d = 1e12;
  auto index = -1;

  // measured landmark
  Vector2D meas_m(m.x, m.y);
  // compute distance to every landmark
  for(unsigned int i = 0; i < lm.size(); i++)
  {
    // known landmark
    Vector2D know_m(lm.at(i)(0), lm.at(i)(1));
    // distance from known landmark to measurement
    const auto d = pointDistance(know_m, meas_m);

    // update if smaller
    if (d < smallest_d)
    {
      smallest_d = d;
      index = i;
    }
  }

  // std::cout << "smallest_d: " << smallest_d << std::endl;
  // std::cout << "known_dist_thresh: " << known_dist_thresh << std::endl;

  // within distance threshold
  if (smallest_d < known_dist_thresh)
  {
    return index;
  }

  return -1;
}


void EKF::measRobotToMap(const std::vector<Vector2D> &meas, std::vector<LM> &lm_meas) const
{
  // TODO: check if conversion to polar is correct

  for(unsigned int i = 0; i < meas.size(); i++)
  {
    const auto mx = meas.at(i).x;
    const auto my = meas.at(i).y;

    // to polar coordinates
    lm_meas.at(i).r = std::sqrt(mx * mx + my *my);
    lm_meas.at(i).b = normalize_angle_PI(std::atan2(my, mx) - normalize_angle_PI(state(0)));

    // // frame: robot -> map
    lm_meas.at(i).x = mx + state(1);
    lm_meas.at(i).y = my + state(2);
  }
}



} // end namespace
