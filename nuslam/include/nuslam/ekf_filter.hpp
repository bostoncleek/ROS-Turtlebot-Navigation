#ifndef FILTER_HPP
#define FILTER_HPP
/// \file
/// \brief EKF SLAM with known and unknown correspondence
#include <eigen3/Eigen/Dense>

#include <cmath>
#include <iosfwd>
#include <vector>

#include <rigid2d/utilities.hpp>
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
  using rigid2d::Transform2D;
  using rigid2d::sampleMultivariateDistribution;


  /// \brief Compose distance from 1.0 to the next largest double-precision number
  /// \param x - the double to query
  /// returns - the distance
  double eps(double x);

  /// \brief Determine whether A is symmetric positive definite
  /// \param A - matrix to test
  /// \returns true is A is SPD
  bool isSPD(const Ref<MatrixXd> A);

  /// \brief Compose nearest SPD matrix to A
  /// \param A - covariance matrix that may not be SPD
  /// A_hat[out] - nearest SPD matrix to A
  void nearestSPD(const Ref<MatrixXd> A, Ref<MatrixXd> A_hat);


  /// \brief Stores data for a landmark
  struct LM
  {
    // x/y location in map frame
    double x = 0.0;
    double y = 0.0;

    // range and bearing relative to robot
    double r = 0.0;
    double b = 0.0;
  };



  class EKF
  {
  public:
    /// \brief Construct EKF
    /// \param num_lm - number of landmarks used in filter
    /// \param md_max - mahalanobis distance threshold for adding new landmark
    /// \param md_min - mahalanobis distance threshold for updating landmark
    EKF(int num_lm, double md_max, double md_min);

    /// \brief Updates the stated vector
    /// \param meas - x/y coordinates of landmarks in the robot frame
    /// \param u - twist from odometry given wheel velocities
    void SLAM(const std::vector<Vector2D> &meas, const Twist2D &u);

    /// \brief Updates the stated vector
    /// \param meas - x/y coordinates of landmarks in the robot frame
    /// \param u - twist from odometry given wheel velocities
    void knownCorrespondenceSLAM(const std::vector<Vector2D> &meas, const Twist2D &u);

    /// \brief Get currnet Robot stated
    /// \returns Transform from map to robot
    Transform2D getRobotState() const;

    /// \brief Get the estimates (x,y) of each landmark
    ///        Does not return landmarks at (0,0) because we assume that the
    ///        robot starts there
    /// map[out] - vector of landmarks position
    void getMap(std::vector<Vector2D> &map) const;


  private:
    /// \brief Initialize state vector, state covariance matrix,
    ///       motion and sensor model noise, and process noise
    ///       assume we know where the robot is but
    ///       do not the where the landmarks are
    ///       assume robot starts at (0,0,0) and
    ///       all landmarks are at (0,0)
    void initFilter();

    /// \brief Estimates the robot pose based on odometry
    /// \param u - twist from odometry given wheel velocities (dtheta, dx, dy=0)
    /// state_bar[out] - estimated state vector
    void motionUpdate(const Twist2D &u, Ref<VectorXd> state_bar) const;

    /// \brief Update the uncertainty in the robots pose
    ///        and for the landmark locations
    /// \param u - twist from odometry given wheel velocities
    /// \param sigma_bar - predicited covariance matrix
    void uncertaintyUpdate(const Twist2D &u, Ref<MatrixXd> sigma_bar) const;

    /// \brief Compose the measurement jacobian
    /// \param j - correspondence id
    /// \param state_bar- estimated state vector
    /// H[out] - the measurement jacobian
    void measurementJacobian(const int j, const Ref<VectorXd> state_bar, Ref<MatrixXd> H) const;

    /// \brief Predicted range and bering given the current state vector
    /// \param j - correspondence id
    /// \param state_bar- estimated state vector
    /// \returns the expected range and bearing of a landmark (r,b)
    Vector2d predictedMeasurement(const int j, const Ref<VectorXd> state_bar) const;

    /// \brief Transform landmark (x,y) into frame of map
    /// \param meas - landmarks (x,y) and (r,b) in frame of robot
    void measRobotToMap(const std::vector<Vector2D> &meas, std::vector<LM> &lm_meas) const;

    /// \brief Initialize a new landmark that has not been
    ///        observed before
    /// \param m - measurement of a landmark in the map frame
    /// \param j - correspondence id
    /// state_bar[out] - estimated state vector
    void newLandmark(const LM &m, const int j, Ref<VectorXd> state_bar) const;

    int n;                         // max number of landmarks, determines state size
    int N;                         // number of landmarks in state vector
    int L;                         // number of landmarks observed
    int state_size;                // size of state vector

    double dmax, dmin;             // max and min mahalanobis distance thresholds

    // (theta, x, y)
    VectorXd state;                // state vector for robot and landmarks
    MatrixXd state_cov;            // state covariance for robot and landmarks

    MatrixXd motion_noise;         // noise in the motion model
    MatrixXd measurement_noise;    // noise in measurement model
    MatrixXd process_noise;        // process noise matrix

    // landmark j in state
    std::vector<int> lm_j;



  };

} // end namespace


#endif
