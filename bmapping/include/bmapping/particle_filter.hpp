#ifndef PARTICLE_FILTER_GUARD_HPP
#define PARTICLE_FILTER_GUARD_HPP
/// \file
/// \brief Particle filter for 2D occupancy grid localization and mapping

#include <Eigen/Dense>
#include <Eigen/Core>

#include <cmath>
#include <iosfwd>
#include <vector>
#include <random>

#include <rigid2d/rigid2d.hpp>
#include <rigid2d/diff_drive.hpp>
#include "bmapping/cloud_alignment.hpp"
#include "bmapping/sensor_model.hpp"
#include "bmapping/grid_mapper.hpp"


namespace bmapping
{
  using Eigen::MatrixXd;
  using Eigen::VectorXd;
  using Eigen::Vector2d;
  using Eigen::Vector3d;
  using Eigen::Ref;

  using rigid2d::Twist2D;
  using rigid2d::Vector2D;
  using rigid2d::Pose;
  using rigid2d::Transform2D;
  using rigid2d::TransformData2D;
  using rigid2d::deg2rad;
  using rigid2d::almost_equal;
  using rigid2d::normalize_angle_PI;

  /// \brief Returns a random number engine
  std::mt19937_64 &getTwister();

  /// \brief samples a standard normal distribution
  /// \param n - number of samples
  /// \returns - random sample
  VectorXd sampleStandardNormal(int n);

  /// \brief samples a multivariate standard normal distribution
  /// \param cov - covariance matrix
  /// \returns - random samples
  VectorXd sampleMultivariateDistribution(const Ref<MatrixXd> cov);

  /// \brief samples a multivariate standard normal distribution
  /// \param mu - distribution mean
  /// \param cov - distribution covariance matrix
  /// \returns - random samples
  VectorXd sampleMultivariateDistribution(const Ref<VectorXd> mu, const Ref<MatrixXd> cov);




  /// \brief Represents a particle
  struct Particle
  {
    // particles weight
    double weight;

    // the map
    GridMapper grid;

    // current pose estimate
    Vector3d pose;

    // previous pose estimate
    Vector3d prev_pose;

    /// \brief Initialize a particle
    /// \param w - weight
    /// \mapper - occupancy grid mapper
    /// \ps - particles pose
    Particle(double w, const GridMapper &mapper, const Vector3d &ps)
                : weight(w),
                  grid(mapper),
                  pose(ps),
                  prev_pose(ps)
                  {}
  };


  /// \brief Particle filter for grid based SLAM
  class ParticleFilter
  {
  public:
    /// \brief Initialize the filter
    /// \param num_particles - number of initial particles
    /// \param k - number of samples around mode
    /// \param pose_init - initial pose
    /// \param scan_matcher - iterative closes point
    ParticleFilter(int num_particles,
                   int k,
                   double srr,
                   double srt,
                   double str,
                   double stt,
                   double motion_noise_theta,
                   double motion_noise_x,
                   double motion_noise_y,
                   double sample_range_theta,
                   double sample_range_x,
                   double sample_range_y,
                   double scan_likelihood_min,
                   double scan_likelihood_max,
                   double pose_likelihood_min,
                   double pose_likelihood_max,
                   ScanAlignment &scan_matcher,
                   const Transform2D &pose,
                   const GridMapper &mapper);

    /// \brief Updates the particle set and the occupancy grid
    /// \param scan - recent laser scan in the robot frame
    /// \param u - twist from odometry given wheel velocities
    void SLAM(const std::vector<float> &scan, const Twist2D &u,
              const Pose &cur_odom, const Pose &prev_odom);

    /// \brief Pose of particle with highest weight
    /// \returns Transform from map to robot
    Transform2D getRobotState();

    /// \brief Map of particle with highest weight
    /// map[out] - new map
    void newMap(std::vector<int8_t> &map);


  private:
    /// \brief Initialize the particle set to begin with
    /// \param mapper - 2D occupancy grid mapping
    /// \param pose - starting pose
    void initParticleSet(const GridMapper &mapper, const Transform2D &pose);

    /// \brief draw a random sample from  x' ~ P(x'|x,u)
    /// \param u - twist from odometry given wheel velocities
    /// pose[out] updated pose
    void sampleMotionModel(const Twist2D &u, Ref<Vector3d> pose);

    // /// \brief Pose likelihood P(x'|x,u)
    // /// \param cur_pose - current pose
    // /// \param prev_pose - previous pose
    // /// \returns likelihood of current pose;
    // double poseLikelihoodTwist(const Ref<Vector3d> cur_pose, const Ref<Vector3d> prev_pose, const Twist2D &u);


    /// \brief Pose likelihood P(x'|x,u)
    /// \param cur_pose - current pose
    /// \param prev_pose - previous pose
    /// \returns likelihood of current pose;
    double poseLikelihoodOdom(const Ref<Vector3d> cur_pose, const Ref<Vector3d> prev_pose,
                              const Ref<Vector3d> cur_odom, const Ref<Vector3d> prev_odom);


    /// \brief Normalize all particle's weights
    void normalizeWeights();

    /// \brief Compose the number of effective particles
    /// \returns true if need to resample
    bool effectiveParticles();

    /// \bried Resample particles
    void lowVarianceResampling();

    /// \brief Samples k poses around mode composed by ICP
    /// \param T - current pose of particle based on icp
    /// sampled_poses[out] - poses aroud mode
    void sampleMode(const Transform2D &T, std::vector<Vector3d> &sampled_poses);


    /// \brief Compose the gaussian distribution from which to sample new
    ///        particles pose from.
    /// \param sampled_poses - poses sampled from the distribution around the
    ///                        transform from ICP
    /// \param particle - the current particle to update
    /// \param scan - recent lidar data
    /// \param cur_odom - recent odometry pose
    /// \param prev_odom - previous odometry pose
    /// \param mu - gaussian proposal mean
    /// \param sigma - gaussian proposal covariance
    /// \param eta - normalization factor
    void gaussianProposal(std::vector<Vector3d> &sampled_poses,
                          Particle &particle,
                          const std::vector<float> &scan,
                          const Ref<Vector3d> cur_odom,
                          const Ref<Vector3d> prev_odom,
                          Ref<Vector3d> mu,
                          Ref<MatrixXd> sigma,
                          double &eta);

    /// \brief Compose the initial guess for ICP based on odometry
    /// \param cur_odom - recent odometry pose
    /// \param prev_odom - previous odometry pose
    /// /return transform of initial guess
    Transform2D icpInitGuess(const Ref<Vector3d> cur_odom, const Ref<Vector3d> prev_odom);


    int num_particles_;                                             // number of particles
    int k_;                                                         // number of samples around mode from ICP

    double srr_, srt_, str_, stt_;                                  // noise parameters for pose likelihood
    double motion_noise_theta_, motion_noise_x_, motion_noise_y_;    // motion model sampling noise
    double sample_range_theta_, sample_range_x_, sample_range_y_;    // sample range for ICP distribution

    double scan_likelihood_min_, scan_likelihood_max_;              // limits on scan likelihood
    double pose_likelihood_min_, pose_likelihood_max_;              // limits on pose likelihood

    double normal_sqrd_sum_;                                        // normalized squared sum of particle weights

    ScanAlignment scan_matcher_;                                    // ICP
    std::vector<Particle> particle_set_;                            // set of particles
    MatrixXd motion_noise_;                                         // noise in the motion model
    MatrixXd sample_range_;                                         // range for sampling mode of transform from ICP

  };


} // end namespace



#endif
