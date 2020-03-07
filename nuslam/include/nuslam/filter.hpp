#ifndef FILTER_HPP
#define FILTER_HPP
/// \file
/// \brief

#include <Eigen/Dense>
#include <Eigen/Core>

#include <cmath>
#include <iosfwd>
#include <vector>
#include <random>

#include <rigid2d/diff_drive.hpp>



namespace nuslam
{
  using Eigen::MatrixXd;
  using Eigen::VectorXd;
  using rigid2d::Twist2D;
  using rigid2d::Vector2D;
  using rigid2d::Pose;


  /// \brief Returns a random number engine
  std::mt19937_64 &getTwister();

  /// \brief samples a standard normal distribution
  /// \param n - number of samples
  /// \returns - random sample
  VectorXd sampleStandardNormal(int n);

  /// \brief samples a multivariate standard normal distribution
  /// \param cov - covariance noise matrix
  /// \returns - random samples
  VectorXd sampleMultivariateDistribution(MatrixXd &cov);



  class EKF
  {
  public:
    EKF();

    void initState(Pose pose, std::vector<Vector2D> lm);

    void initCov(int num_lm);

    /// \brief Updates the robot pose based on odometry
    /// \param u - change in the odometry pose (dtheta, dx, dy=0)
    void motionUpdate(Twist2D u);


  private:
    // (theta, x, y)
    MatrixXd state;             // state vector for robot and landmarks
    MatrixXd state_cov;         // state covariance for robot and landmarks

    // motion model noise
    MatrixXd motion_noise;      // noise in the motion model


  };





} // end namespace


#endif
