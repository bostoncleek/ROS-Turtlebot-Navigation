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

#include <rigid2d/rigid2d.hpp>
#include "nuslam/landmarks.hpp"



namespace nuslam
{
  using Eigen::MatrixXd;
  using Eigen::VectorXd;
  using Eigen::Vector2d;
  using Eigen::Vector3d;
  using Eigen::Ref;
  using rigid2d::Twist2D;
  using rigid2d::Vector2D;
  using rigid2d::almost_equal;
  using rigid2d::normalize_angle_PI;

  /// \brief Returns a random number engine
  std::mt19937_64 &getTwister();

  /// \brief samples a standard normal distribution
  /// \param n - number of samples
  /// \returns - random sample
  VectorXd sampleStandardNormal(int n);

  /// \brief samples a multivariate standard normal distribution
  /// \param cov - covariance noise matrix
  /// \returns - random samples
  VectorXd sampleMultivariateDistribution(const MatrixXd &cov);



  struct LM
  {
    double x = 0.0;
    double y = 0.0;

    double r = 0.0;
    double b = 0.0;
  };



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

    /// \brief Set known landmarks
    /// \param lm - (x,y,id) in map for landmarks
    void setKnownLandamrks(const std::vector<Vector3d> &landmarks);

    /// \brief Updates the robot pose based on odometry
    /// \param u - twist from odometry given wheel velocities (dtheta, dx, dy=0)
    void motionUpdate(const Twist2D &u);

    /// \brief Update the uncertainty in the robots pose
    ///        and for the landmark locations
    /// \param u - twist from odometry given wheel velocities
    /// \param sigma_bar - predicited covariance matrix
    void uncertaintyUpdate(const Twist2D &u, Ref<MatrixXd> sigma_bar) const;

    /// \brief Compose the measurement jacobian
    /// \param dx - relative x-distance from robot to landmark
    /// \param dj - relative j-distance from robot to landmark
    /// \param j - correspondence id
    /// H[out] - the measurement jacobian
    void measurementJacobian(const int j, Ref<MatrixXd> H) const;

    /// \brief Predicted range and bering given the current state vector
    /// \param j - correspondence id
    /// \returns the expected range and bearing of a landmark (r,b)
    Vector2d predictedMeasurement(const int j) const;

    /// \brief Access the (x,y) position of landmark in the map
    /// \param j - correspondence id
    /// \returns -  (x,y) position of landmark with id j
    Vector2d landmarkState(const int j) const;

    /// \brief Initialize a new landmark that has not been
    ///        observed before
    /// \param m - measurement of a landmark in the map frame
    /// \param j - correspondence id
    void newLandmark(const LM &m, const int j);

    /// \brief Updates the stated vector
    /// \param meas - x/y coordinates of landmarks in the robot frame
    /// \param u - twist from odometry given wheel velocities
    void knownCorrespondenceSLAM(const std::vector<Vector2D> &meas, const Twist2D &u);


    /// \brief Searches for the corresponding landmark id (j)
    /// \param m - measurement of a landmark in the map frame
    /// returns - landmark id and -1 if not found
    int findKnownCorrespondence(const LM &m) const;


    /// \brief Transform landmark (x,y) into frame of map
    /// \param meas - landmarks (x,y) and (r,b) in frame of robot
    void measRobotToMap(const std::vector<Vector2D> &meas, std::vector<LM> &lm_meas) const;



  private:
    int n;                         // max number of landmarks, determines state size
    // int n_obs;                     // number of landmarks observed so far
    int state_size;                // size of state vector

    double known_dist_thresh;      // distance threshold for known correspondences

    // (theta, x, y)
    VectorXd state;                // state vector for robot and landmarks
    MatrixXd state_cov;            // state covariance for robot and landmarks

    MatrixXd motion_noise;         // noise in the motion model
    MatrixXd measurement_noise;    // noise in measurement model
    MatrixXd process_noise;        // process noise matrix

    // for known correspondences
    std::vector<Vector3d> lm;

    // landmark j observed
    std::vector<int> lm_j;

  };





} // end namespace


#endif
