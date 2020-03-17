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
  /// \param cov - covariance noise matrix
  /// \returns - random samples
  VectorXd sampleMultivariateDistribution(const MatrixXd &cov);


  /// \brief Represents a particle
  struct Particle
  {
    // particles weight
    double weight;

    // the map
    GridMapper grid;

    // pose estimate
    Pose pose;

    /// \brief Initialize a particle
    /// \param w - weight
    /// \mapper - occupancy grid mapper
    /// \ps - particles pose
    Particle(double w, const GridMapper &mapper, const Pose &ps)
                : weight(w),
                  grid(mapper),
                  pose(ps)
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
                   ScanAlignment &scan_matcher,
                   const Pose &pose,
                   const GridMapper &mapper);

    /// \brief Updates the particle set and the occupancy grid
    /// \param scan - recent laser scan in the robot frame
    /// \param u - twist from odometry given wheel velocities
    void SLAM(const std::vector<float> &scan, const Twist2D &u);

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
    void initParticleSet(const GridMapper &mapper, const Pose &pose);

    /// \brief draw a random sample from  x' ~ P(x'|x,u)
    /// \param u - twist from odometry given wheel velocities
    /// pose[out] updated pose
    void sampleMotionModel(const Twist2D &u, Pose &pose);

    /// \brief Normalize all particle's weights
    void normalizeWeights();

    /// \brief Compose the number of effective particles
    /// \returns true if need to resample
    bool effectiveParticles();

    /// \bried Resample particles
    void lowVarianceResampling();


    int num_particles_;                               // number of particles
    int k_;                                           // number of samples around mode from ICP
    double normal_sqrd_sum_;                          // normalized squared sum of particle weights

    ScanAlignment scan_matcher_;                      // ICP
    std::vector<Particle> particle_set_;              // set of particles
    MatrixXd motion_noise_;                            // noise in the motion model


  };


} // end namespace



#endif
