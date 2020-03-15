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





EKF::EKF(int num_lm, double md_max, double md_min)
                    : n(num_lm),
                      N(0),
                      dmax(md_max),
                      dmin(md_min)
{
  // total state vector size robot plus landmarks
  state_size = 3 + 2 * n;

  initFilter();
}


// void EKF::setKnownLandamrks(const std::vector<Vector3d> &landmarks)
// {
//   lm = landmarks;
// }


/// \brief Updates the stated vector
/// \param meas - x/y coordinates of landmarks in the robot frame
/// \param u - twist from odometry given wheel velocities
void EKF::SLAM(const std::vector<Vector2D> &meas, const Twist2D &u)
{
  // 1) motion model
  VectorXd state_bar = VectorXd::Zero(state_size);
  motionUpdate(u, state_bar);


  // 2) propagate uncertainty
  MatrixXd sigma_bar = MatrixXd::Zero(state_size, state_size);
  uncertaintyUpdate(u, sigma_bar);

  // 3) update state based on observations
  // measurements come in as (x,y) in robot frame
  // convert them to range and bearing
  // convert landmark positions in robot frame to map frame
  std::vector<LM> lm_meas(meas.size());
  measRobotToMap(meas, lm_meas);

  std::cout << "--------------------------------------" << std::endl;

  // for(const auto &m : lm_meas)
  for(unsigned int i = 0; i < n;/*lm_meas.size();*/ i++)
  {
    LM m = lm_meas.at(i);

    // check if outside search radius
    if(std::isnan(m.x) && std::isnan(m.y))
    {
      continue;
    }

    std::cout << "Current Number of landmarks: " << N << std::endl;


    // logic for adding first arguement
    std::vector<double> distances;
    if (N == 0)
    {
      // only place one element set to a large number
      // so the first landmark is added
      distances.push_back(1e12);
    }
    else
    {
      distances.reserve(N);
    }


    // compute mahalanobis distance to each landmark
    std::cout << "---- mahalanobis loop ----" << std::endl;
    for(int k = 0; k < N; k++)
    {

      // predicted measurement z_hat (r,b)
      Vector2d z_hat = predictedMeasurement(k, state_bar);
      std::cout << "z_hat" << std::endl;
      std::cout << z_hat << std::endl;


      // measurement jacobian
      MatrixXd H = MatrixXd::Zero(2, state_size);
      measurementJacobian(k, state_bar, H);
      std::cout << "H" << std::endl;
      std::cout << H << std::endl;


      // Psi
      MatrixXd Psi = MatrixXd::Zero(2, 2);
      Psi = H * sigma_bar * H.transpose() + measurement_noise;
      std::cout << "Psi" << std::endl;
      std::cout << Psi << std::endl;


      // difference in measurements delta_z (r,b)
      Vector2d delta_z;
      delta_z(0) = m.r - z_hat(0);
      delta_z(1) = normalize_angle_PI(normalize_angle_PI(m.b) - normalize_angle_PI(z_hat(1)));
      std::cout << "delta_z" << std::endl;
      std::cout << delta_z << std::endl;


      // mahalanobis distance
      const auto d = delta_z.transpose() * Psi.inverse() * delta_z;

      distances.push_back(d);
      std::cout << "mahalanobis distance: " << d << std::endl;

      std::cout << "Determinant: " << Psi.determinant() << std::endl;
      std::cout << "Psi.inverse()" << std::endl;
      std::cout << Psi.inverse() << std::endl;

      if (d < 0)
      {
        throw std::invalid_argument("WARNING mahalanobis distance is negative");
      }

    } // end inner for loop
    std::cout << "---- mahalanobis loop ----" << std::endl;


    // find index of d* (min mahalanobis distance)
    auto j = std::min_element(distances.begin(), distances.end()) - distances.begin();
    const auto dstar = distances.at(j);
    std::cout << "landmark ID: " << j << std::endl;
    std::cout << "Min mahalanobis distance: " << dstar << std::endl;


    if (dstar < dmin or dstar > dmax)
    {
      // d* is bellow d*_min update existing landmark
      if (dstar < dmin)
      {
        std::cout << "Updating existing landmark: " << j << std::endl;
      }

      // d* is above d*_max add new landmark
      // increment number of landmarks
      else if (dstar > dmax)
      {
        // max number of landmarks
        if ((N + 1) <= n)
        {
          // set new ID to N
          // because we index from 0
          j = N;

          std::cout << "Adding new landmark ID: " << j << std::endl;
          newLandmark(m, j, state_bar);

          N++;
        }

        else
        {
          throw std::invalid_argument("WARNING Max number of landmarks reached");
          // std::cout << "WARNING Max number of landmarks reached" << std::endl;
        }
      }

      // update based on index d* (j)
      // 4) predicted measurement z_hat (r,b)
      Vector2d z_hat = predictedMeasurement(j, state_bar);
      std::cout << "z_hat" << std::endl;
      std::cout << z_hat << std::endl;


      // 5) measurement jacobian
      MatrixXd H = MatrixXd::Zero(2, state_size);
      measurementJacobian(j, state_bar, H);
      std::cout << "H" << std::endl;
      std::cout << H << std::endl;

      // 6) kalman gain
      MatrixXd temp = MatrixXd::Zero(2,2);
      temp = H * sigma_bar * H.transpose() + measurement_noise;
      std::cout << "temp" << std::endl;
      std::cout << temp << std::endl;

      MatrixXd temp_inv = MatrixXd::Zero(2,2);
      temp_inv = temp.inverse();
      // std::cout << "Inverse" << std::endl;
      // std::cout << temp_inv << std::endl;


      MatrixXd K = MatrixXd::Zero(state_size, 2);
      K = sigma_bar * H.transpose() * temp_inv;
      std::cout << "kalman Gain" << std::endl;
      std::cout << K << std::endl;


      // 7) Update the state vector
      // difference in measurements delta_z (r,b)
      Vector2d delta_z;
      delta_z(0) = m.r - z_hat(0);
      delta_z(1) = normalize_angle_PI(normalize_angle_PI(m.b) - normalize_angle_PI(z_hat(1)));
      std::cout << "delta_z" << std::endl;
      std::cout << delta_z << std::endl;

      state_bar += K * delta_z;
      std::cout << "state_bar" << std::endl;
      std::cout << state_bar << std::endl;


      // 8) Update covariance sigma bar
      MatrixXd I = MatrixXd::Identity(state_size, state_size);
      sigma_bar = (I - (K * H)) * sigma_bar;


    } // end update/add landmark if

    else
    {
      std::cout << "!!!!!!! Dead Band Zone !!!!!!!" << std::endl;
    }
  } // end  outer loop


   // Update state vector
  state = state_bar;
  std::cout << "state" << std::endl;
  std::cout << state << std::endl;

  //  Update covariance
  state_cov = sigma_bar;
  // std::cout << "Covariance" << std::endl;
  // std::cout << state_cov << std::endl;

  std::cout << "--------------------------------------" << std::endl;
}



void EKF::knownCorrespondenceSLAM(const std::vector<Vector2D> &meas, const Twist2D &u)
{
  // 1) motion model
  VectorXd state_bar = VectorXd::Zero(state_size);
  motionUpdate(u, state_bar);


  // 2) propagate uncertainty
  MatrixXd sigma_bar = MatrixXd::Zero(state_size, state_size);
  uncertaintyUpdate(u, sigma_bar);

  // 3) update state based on observations
  // measurements come in as (x,y) in robot frame
  // convert them to range and bearing
  // convert landmark positions in robot frame to map frame
  std::vector<LM> lm_meas(meas.size());
  measRobotToMap(meas, lm_meas);

  std::cout << "--------------------------------------" << std::endl;

  // for(const auto &m : lm_meas)
  for(unsigned int i = 0; i < lm_meas.size(); i++)
  // LM m = lm_meas.at(0);
  {
    // new measurement
    LM m = lm_meas.at(i);

    // check if outside search radius
    if(std::isnan(m.x) && std::isnan(m.y))
    {
      continue;
    }

    // find correspondence id is the index the measurement comes in at
    int j = i;//0;
    // std::cout << j << std::endl;


    // landmark has not been scene before
    if (std::find(lm_j.begin(), lm_j.end(), j) == lm_j.end())
    {
      // std::cout << "new landmark id: " << j << std::endl;
      // add id to observed list
      // init new landmark with measurement
      lm_j.push_back(j);
      newLandmark(m, j, state_bar);
    }


    // 4) predicted measurement z_hat (r,b)
    Vector2d z_hat = predictedMeasurement(j, state_bar);
    // std::cout << "z_hat" << std::endl;
    // std::cout << z_hat << std::endl;


    // 5) measurement jacobian
    MatrixXd H = MatrixXd::Zero(2, state_size);
    measurementJacobian(j, state_bar, H);
    // std::cout << "H" << std::endl;
    // std::cout << H << std::endl;

    // 6) kalman gain
    MatrixXd temp = MatrixXd::Zero(2,2);
    temp = H * sigma_bar * H.transpose() + measurement_noise;

    // std::cout << "temp" << std::endl;
    // std::cout << temp << std::endl;

    MatrixXd temp_inv = MatrixXd::Zero(2,2);
    temp_inv = temp.inverse();
    // std::cout << "Determinant: " << temp.determinant() << std::endl;
    // std::cout << "Inverse" << std::endl;
    // std::cout << temp_inv << std::endl;


    MatrixXd K = MatrixXd::Zero(state_size, 2);
    K = sigma_bar * H.transpose() * temp_inv;
    // std::cout << "kalman Gain" << std::endl;
    // std::cout << K << std::endl;


    // 7) Update the state vector
    // difference in measurements delta_z (r,b)
    Vector2d delta_z;
    delta_z(0) = m.r - z_hat(0);
    delta_z(1) = normalize_angle_PI(normalize_angle_PI(m.b) - normalize_angle_PI(z_hat(1)));
    // std::cout << "delta_z" << std::endl;
    // std::cout << delta_z << std::endl;

    state_bar += K * delta_z;
    // std::cout << "state_bar" << std::endl;
    // std::cout << state_bar << std::endl;


    // 8) Update covariance sigma bar
    MatrixXd I = MatrixXd::Identity(state_size, state_size);
    sigma_bar = (I - (K * H)) * sigma_bar;

  } // end loop

  // 9) Update state vector
  state = state_bar;
  // std::cout << "state" << std::endl;
  // std::cout << state << std::endl;

  // 9) Update covariance
  state_cov = sigma_bar;
  // std::cout << "Covariance" << std::endl;
  // std::cout << state_cov << std::endl;

  std::cout << "--------------------------------------" << std::endl;
}


Transform2D EKF::getRobotState()
{
  Vector2D vmr(state(1), state(2));
  Transform2D Tmr(vmr, state(0));
  return Tmr;
}


void EKF::getMap(std::vector<Vector2D> &map)
{
  map.reserve(n);
  for(auto i = 0; i < n; i++)
  {
    const auto jx = 2*i + 3;
    const auto jy = 2*i + 4;

    Vector2D marker(state(jx), state(jy));

    // assume no landmarks are here at (0,0)
    // because robot starts here
    if (!almost_equal(marker.x, 0.0) and !almost_equal(marker.y, 0.0))
    {
      map.push_back(marker);
    }
  }
}


void EKF::initFilter()
{
  ////////////////////////////////////////////

  // init state vector
  // pose is (theta, x, y)
  // set pose to (0,0,0)
  state = VectorXd::Zero(state_size);
  // state(0) = 0.174533;
  // state(1) = 0.3;
  // std::cout << state << std::endl;

  ////////////////////////////////////////////

  // init state covariance
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

  ////////////////////////////////////////////

  // init motion noise
  motion_noise = MatrixXd::Zero(3,3);
  motion_noise(0,0) = 1e-10;  // theta var
  motion_noise(1,1) = 1e-5;   // x var
  motion_noise(2,2) = 1e-5;   // y var
  // std::cout << motion_noise << std::endl;

  ////////////////////////////////////////////

  // init measurement noise
  measurement_noise = MatrixXd::Zero(2,2);
  measurement_noise(0,0) = 1e-5;   // r var
  measurement_noise(1,1) = 1e-5;   // b var
  // std::cout << measurement_noise << std::endl;

  ////////////////////////////////////////////

  // init process noise
  process_noise = MatrixXd::Zero(state_size, state_size);
  process_noise(0,0) = motion_noise(0,0);
  process_noise(1,1) = motion_noise(1,1);
  process_noise(2,2) = motion_noise(2,2);
  // std::cout << process_noise << std::endl;
}


void EKF::motionUpdate(const Twist2D &u, Ref<VectorXd> state_bar)
{
  // set estimated state equal to current state and then update it
  state_bar = state;

  // sample noise
  VectorXd w = sampleMultivariateDistribution(motion_noise);


  // update robot pose based on odometry
  if (almost_equal(u.w, 0.0))
  {
    // update theta
    state_bar(0) = normalize_angle_PI(state_bar(0) + w(0));
    // update x
    state_bar(1) += u.vx * std::cos(state_bar(0)) + w(1);
    // update y
    state_bar(2) += u.vx * std::sin(state_bar(0)) + w(2);
  }

  else
  {
    // update theta
    state_bar(0) = normalize_angle_PI(state_bar(0) + u.w + w(0));
    // update x
    state_bar(1) += (-u.vx / u.w) * std::sin(state_bar(0)) + \
                            (u.vx / u.w) * std::sin(state_bar(0) + u.w) + w(1);
    // update y
    state_bar(2) += (u.vx / u.w) * std::cos(state_bar(0)) - \
                            (u.vx / u.w) * std::cos(state_bar(0) + u.w) + w(2);
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



void EKF::measurementJacobian(const int j, const Ref<VectorXd> state_bar, Ref<MatrixXd> H) const
{
  const auto jx = 2*j + 3;
  const auto jy = 2*j + 4;

  const auto dx = state_bar(jx) - state_bar(1);
  const auto dy = state_bar(jy) - state_bar(2);

  const auto q = dx*dx + dy*dy;
  const auto sqrt_q = std::sqrt(q);


  // row 1
  H(0,0) = 0.0;
  H(0,1) = -dx / sqrt_q;
  H(0,2) = -dy / sqrt_q;

  H(0,jx) = dx / sqrt_q;
  H(0,jy) = dy / sqrt_q;


  // row 2
  H(1,0) = -1.0;
  H(1,1) = dy / q;
  H(1,2) = -dx / q;

  H(1,jx) = -dy /q;
  H(1,jy) = dx / q;


  // MatrixXd F = MatrixXd::Zero(5, state_size);
  // MatrixXd h = MatrixXd::Zero(2, 5);
  //
  // // upper left is diagonal for pose
  // F(0,0) = 1;
  // F(1,1) = 1;
  // F(2,2) = 1;
  //
  // // lower right diagonal
  // F(3, 2*j + 3) = 1;
  // F(4, 2*j + 4) = 1;
  // // std::cout << F << std::endl;
  //
  //
  // // // row 1
  // h(0,0) = 0.0;
  // h(0,1) = -dx / sqrt_q;
  // h(0,2) = -dy / sqrt_q;
  // h(0,3) = dx / sqrt_q;
  // h(0,4) = dy / sqrt_q;
  //
  // // // row 2
  // h(1,0) = -1.0;
  // h(1,1) = dy / q;
  // h(1,2) = -dx / q;
  // h(1,3) = -dy /q;
  // h(1,4) = dx / q;
  //
  //
  // H = h * F;
  // std::cout << H << std::endl;
}


Vector2d EKF::predictedMeasurement(const int j, const Ref<VectorXd> state_bar) const
{
  // index of jth correspondence
  // first 3 indecis are for (theta, x, y)
  const auto jx = 2*j + 3;
  const auto jy = 2*j + 4;

  // change in x landmark to robot
  const auto delta_x = state_bar(jx) - state_bar(1);
  const auto delta_y = state_bar(jy) - state_bar(2);

  // measurement noise
  VectorXd v = sampleMultivariateDistribution(measurement_noise);

  Vector2d z_hat;

  // predicted range
  z_hat(0) = std::sqrt(delta_x * delta_x + delta_y * delta_y) + v(0);
  // predicted bearing
  z_hat(1) = normalize_angle_PI(std::atan2(delta_y, delta_x) - normalize_angle_PI(state_bar(0)) + v(1));


  return z_hat;
}


void EKF::measRobotToMap(const std::vector<Vector2D> &meas, std::vector<LM> &lm_meas) const
{
  // TODO: check if conversion to polar is correct

  for(unsigned int i = 0; i < meas.size(); i++)
  {
    const auto mx = meas.at(i).x;
    const auto my = meas.at(i).y;

    // to polar coordinates
    const auto r = std::sqrt(mx * mx + my *my);

    // TODO: check this bearing
    const auto b = normalize_angle_PI(std::atan2(my, mx));// - normalize_angle_PI(state(0)));

    lm_meas.at(i).r = r;
    lm_meas.at(i).b = b;

    // // frame: robot -> map
    lm_meas.at(i).x = state(1) + r * std::cos(b + state(0));
    lm_meas.at(i).y = state(2) + r * std::sin(b + state(0));



    // if (i == 0)
    // {
    //   std::cout << "ID: " << i << " x: " << lm_meas.at(i).x << " y: " << lm_meas.at(i).y << std::endl;
    //   // std::cout << "ID: " << i << " r: " << r << " b: " << b << std::endl;
    // }


  }
}


void EKF::newLandmark(const LM &m, const int j, Ref<VectorXd> state_bar)
{
  // index of jth correspondence
  // first 3 indecis are for (theta, x, y)
  const auto jx = 2*j + 3;
  const auto jy = 2*j + 4;

  state_bar(jx) = state_bar(1) + m.r * std::cos(m.b + state_bar(0));
  state_bar(jy) = state_bar(2) + m.r * std::sin(m.b + state_bar(0));
}



} // end namespace



// kalman gain
// MatrixXd K = MatrixXd::Zero(state_size, 2);
// K = sigma_bar * H_vec.at(l).transpose() * Psi_vec.at(l).inverse();
// std::cout << "kalman Gain" << std::endl;
// std::cout << K << std::endl;
//
// // Update the state vector
// state_bar += K * delta_z_vec.at(l);
// std::cout << "state_bar" << std::endl;
// std::cout << state_bar << std::endl;
//
// // Update covariance sigma bar
// MatrixXd I = MatrixXd::Identity(state_size, state_size);
// sigma_bar = (I - (K * H_vec.at(l))) * sigma_bar;
