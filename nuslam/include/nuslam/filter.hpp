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
  using Eigen::Ref;
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
    EKF(int num_lm);

    /// \brief Initialize state vector
    ///        assume robot starts at (0,0,0) and
    ///        all landmarks are at (0,0)
    void initState();

    /// \brief Initialize the state covariance matrix
    ///        assume we know where the robot is but
    ///        do not the where the landmarks are
    void initCov();

    /// \brief Initialize the Process noise matrix
    void initProcessNoise();

    /// \brief Updates the robot pose based on odometry
    /// \param u - change in the odometry pose (dtheta, dx, dy=0)
    void motionUpdate(const Twist2D &u);

    /// \brief Update the uncertainty in the robots pose
    ///        and for the landmark locations
    void uncertaintyUpdate(const Twist2D &u, Ref<MatrixXd> sigma_bar);


  private:
    int n;                      // number of landmarks

    // (theta, x, y)
    VectorXd state;             // state vector for robot and landmarks
    MatrixXd state_cov;         // state covariance for robot and landmarks

    MatrixXd motion_noise;      // noise in the motion model
    MatrixXd process_noise;    // process noise matrix


  };





} // end namespace


#endif
