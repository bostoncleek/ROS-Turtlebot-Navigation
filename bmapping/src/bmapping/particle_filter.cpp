/// \file
/// \brief Particle filter for 2D occupancy grid localization and mapping

#include <iostream>
#include <algorithm>
#include <stdexcept>
#include <iomanip>

#include "bmapping/particle_filter.hpp"



namespace bmapping
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
  MatrixXd L( cov.llt().matrixL() );

  return L * rand_vec;
}



// public

ParticleFilter::ParticleFilter(int num_particles,
                               int k,
                               ScanAlignment &scan_matcher,
                               const Pose &pose,
                               const GridMapper &mapper)
                                 : num_particles_(num_particles),
                                   k_(k),
                                   normal_sqrd_sum_(0.0),
                                   scan_matcher_(scan_matcher)
{
  // initialize set of particles
  initParticleSet(mapper, pose);

  // init motion noise
  motion_noise_ = MatrixXd::Zero(3,3);
  motion_noise_(0,0) = 1e-10;  // theta var
  motion_noise_(1,1) = 1e-10;   // x var
  motion_noise_(2,2) = 1e-10;   // y var
}



// private

void ParticleFilter::initParticleSet(const GridMapper &mapper, const Pose &pose)
{
  const auto weight = 1.0 / num_particles_;

  particle_set_.reserve(num_particles_);
  for(auto i = 0; i < num_particles_; i++)
  {
    Particle particle(weight, mapper, pose);
    particle_set_.push_back(particle);
  }
}


void ParticleFilter::SLAM(const std::vector<float> &scan, const Twist2D &u)
{
  for(auto &particle: particle_set_)
  {
    // express pose of particle as transform
    Vector2D vec(particle.pose.x, particle.pose.y);
    Transform2D T_pose(vec, particle.pose.theta);

    // draw new pose from distribution
    sampleMotionModel(u, particle.pose);

    // update weight for each particle
    const auto scan_likelihood = particle.grid.likelihoodFieldModel(scan, T_pose);
    particle.weight *= scan_likelihood;


    // update map
    particle.grid.integrateScan(scan, T_pose);

  } // end loop


  normalizeWeights();
  if(effectiveParticles())
  {
    std::cout << "Resampling" << std::endl;
    lowVarianceResampling();
  }

}



Transform2D ParticleFilter::getRobotState()
{
  auto weight = 0.0;
  auto idx = 0;

  for(auto i = 0; i < num_particles_; i++)
  {
    if (particle_set_.at(i).weight > weight)
    {
      weight = particle_set_.at(i).weight;
      idx = i;
    }
  }
  Pose pose = particle_set_.at(idx).pose;

  Vector2D vec(pose.x, pose.y);
  Transform2D T_pose(vec, pose.theta);

  return T_pose;
}


void ParticleFilter::newMap(std::vector<int8_t> &map)
{
  auto weight = 0.0;
  auto idx = 0;

  for(auto i = 0; i < num_particles_; i++)
  {
    if (particle_set_.at(i).weight > weight)
    {
      weight = particle_set_.at(i).weight;
      idx = i;
    }
  }
  particle_set_.at(idx).grid.gridMap(map);
}



void ParticleFilter::sampleMotionModel(const Twist2D &u, Pose &pose)
{
  // sample noise
  VectorXd w = sampleMultivariateDistribution(motion_noise_);

  // update robot pose based on odometry
  if (almost_equal(u.w, 0.0))
  {
    // update theta
    pose.theta = normalize_angle_PI(pose.theta + w(0));
    // update x
    pose.x += u.vx * std::cos(pose.theta) + w(1);
    // update y
    pose.y += u.vx * std::sin(pose.theta) + w(2);
  }

  else
  {
    // update theta
    pose.theta = normalize_angle_PI(pose.theta + u.w + w(0));
    // update x
    pose.x += (-u.vx / u.w) * std::sin(pose.theta) + \
                            (u.vx / u.w) * std::sin(pose.theta + u.w) + w(1);
    // update y
    pose.y += (u.vx / u.w) * std::cos(pose.theta) - \
                            (u.vx / u.w) * std::cos(pose.theta + u.w) + w(2);
  }
}


void ParticleFilter::normalizeWeights()
{
  // TODO: check if sum > 0

  auto sum = 0.0;
  for(const auto &particle: particle_set_)
  {
    sum += particle.weight;
  }

  normal_sqrd_sum_ = 0.0;
  for(auto &particle: particle_set_)
  {
    particle.weight /= sum;
    normal_sqrd_sum_ += std::pow(particle.weight, 2);
  }
}


bool ParticleFilter::effectiveParticles()
{
  std::cout << "Neff: " << static_cast<int> (1.0 / normal_sqrd_sum_) << std::endl;
  return (static_cast<int> (1.0 / normal_sqrd_sum_) < (num_particles_ / 2)) ? true : false;
}


void ParticleFilter::lowVarianceResampling()
{
  // temporary set to store selected particles
  std::vector<Particle> temp_particle_set;

  // randomly generate partioning constant
  VectorXd v = sampleStandardNormal(1);
  const auto r =  (v(0) / static_cast<double> (num_particles_));

  // start with weight of first particle
  auto c = particle_set_.at(0).weight;

  auto i = 0;
  for(auto m = 0; m < num_particles_; m++)
  {
     const auto U = r + static_cast<double> (m * (1.0 / num_particles_));
     while(U > c)
     {
       i++;
       c += particle_set_.at(i).weight;
     }
     temp_particle_set.push_back(particle_set_.at(i));
  }

  particle_set_.clear();
  particle_set_ = temp_particle_set;
}

} // end namespace









// end file
