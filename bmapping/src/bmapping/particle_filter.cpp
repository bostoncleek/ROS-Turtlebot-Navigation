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


VectorXd sampleMultivariateDistribution(const Ref<MatrixXd> cov)
{
  // must be square
  int dim = cov.cols();
  VectorXd rand_vec = sampleStandardNormal(dim);

  // cholesky decomposition
  MatrixXd L( cov.llt().matrixL() );

  return L * rand_vec;
}


VectorXd sampleMultivariateDistribution(const Ref<VectorXd> mu, const Ref<MatrixXd> cov)
{
  // must be square
  int dim = cov.cols();
  VectorXd rand_vec = sampleStandardNormal(dim);

  // cholesky decomposition
  MatrixXd L( cov.llt().matrixL() );

  return (mu + L * rand_vec);
}




// public

ParticleFilter::ParticleFilter(int num_particles,
                               int k,
                               int frequency,
                               ScanAlignment &scan_matcher,
                               const Transform2D &pose,
                               const GridMapper &mapper)
                                 : num_particles_(num_particles),
                                   k_(k),
                                   normal_sqrd_sum_(0.0),
                                   frequency_(frequency),
                                   scan_matcher_(scan_matcher)
{
  // initialize set of particles
  initParticleSet(mapper, pose);

  // init motion noise
  motion_noise_ = MatrixXd::Zero(3,3);
  motion_noise_(0,0) = 1e-10;  // theta var
  motion_noise_(1,1) = 1e-5;   // x var
  motion_noise_(2,2) = 1e-5;   // y var

  // init sample range around mode
  sample_range_ = MatrixXd::Zero(3,3);
  sample_range_(0,0) = 1e-10;  // theta var
  sample_range_(1,1) = 1e-8;   // x var
  sample_range_(2,2) = 1e-8;   // y var


  // Twist2D u;
  // u.w = 0.0;
  // u.vx = 1.0;
  //
  // Vector3d pnew(0.0, 1.0, 0.001);
  // Vector3d pold(0.0, 0.0, 0.0001);
  //
  // double p = poseLikelihood(pnew, pold, u);
  // std::cout << "pose likelihood TEST" << std::endl;
  // std::cout << p << std::endl;

  // Vector3d cur_pose(0.0, 1.0, 0.0);
  // Vector3d prev_pose(0.0, 0.0, 0.0);
  //
  // Vector3d cur_odom(0.0, 1.0, 0.0);
  // Vector3d prev_odom(0.0, 1.0, 0.0);
  //
  // double p = poseLikelihoodOdom(cur_pose, prev_pose, cur_odom, prev_odom);
  // std::cout << "pose likelihood TEST" << std::endl;
  // std::cout << p << std::endl;
}



// private

void ParticleFilter::initParticleSet(const GridMapper &mapper, const Transform2D &pose)
{
  const auto weight = 1.0 / num_particles_;

  particle_set_.reserve(num_particles_);
  for(auto i = 0; i < num_particles_; i++)
  {
    TransformData2D T2d = pose.displacement();
    Vector3d ps(T2d.theta, T2d.x, T2d.y);

    Particle particle(weight, mapper, ps);
    particle_set_.push_back(particle);
  }
}


void ParticleFilter::SLAM(const std::vector<float> &scan, const Twist2D &u,
                          const Pose &cur_odom, const Pose &prev_odom)
{

  // find transform between scans
  Transform2D Ticp;
  // initial guess
  Vector3d cur_od(cur_odom.theta, cur_odom.x, cur_odom.y);
  Vector3d prev_od(prev_odom.theta, prev_odom.x, prev_odom.y);
  Transform2D Tinit = icpInitGuess(cur_od, prev_od);

  // ICP
  bool matcher_success = scan_matcher_.pclICPWrapper(Ticp, Tinit, scan);
  // scan_matcher_.pclICPWrapper(Ticp, Tinit, scan);
  // bool matcher_success = false;


  for(auto &particle: particle_set_)
  {
    // scan matcher fails
    if (!matcher_success)
    {
      // update previous pose of particle
      particle.prev_pose = particle.pose;

      // draw new pose from distribution
      sampleMotionModel(u, particle.pose);

      // express pose of particle as transform
      Vector2D p_vec(particle.pose(1), particle.pose(2));
      Transform2D T_pose(p_vec, particle.pose(0));

      // update weight for each particle
      const auto scan_likelihood = particle.grid.likelihoodFieldModel(scan, T_pose);
      particle.weight *= scan_likelihood;


      // update map
      // particle.grid.integrateScan(scan, T_pose);


      // TEST
      // double p = poseLikelihood(particle.pose, particle.prev_pose, u);
      // Vector3d cur_od(cur_odom.theta, cur_odom.x, cur_odom.y);
      // Vector3d prev_od(prev_odom.theta, prev_odom.x, prev_odom.y);
      //
      // double p = poseLikelihoodOdom(particle.pose, particle.prev_pose, cur_od, prev_od);
      // std::cout << "pose likelihood" << std::endl;
      // std::cout << p << std::endl;
    }

    else
    {
      // particles estimated pose based on ICP
      Vector2D vec(particle.pose(1), particle.pose(2));
      // Transform2D T_x = T_pose;//(vec, particle.pose(0));
      Transform2D T_x(vec, particle.pose(0));
      T_x = T_x * Ticp;


      // sample around mode
      std::vector<Vector3d> sampled_poses;
      sampleMode(T_x, sampled_poses);


      // compute gaussian proposal
      // mu (theta, x, y)
      Vector3d mu(0.0, 0.0, 0.0);
      // covariance
      MatrixXd sigma = MatrixXd::Zero(3,3);
      // normalization factor
      auto eta = 0.0;



      // Vector3d cur_od(cur_odom.theta, cur_odom.x, cur_odom.y);
      // Vector3d prev_od(prev_odom.theta, prev_odom.x, prev_odom.y);

      gaussianProposal(sampled_poses, particle, u, scan, cur_od, prev_od, mu, sigma, eta);

      // std::cout << "sample mu" << std::endl;
      // std::cout << mu << std::endl;
      // std::cout << "sample cov" << std::endl;
      // std::cout << sigma << std::endl;
      // std::cout << "eta: " << std::setprecision(9) << eta << std::endl;


      // sample particles new pose
      Vector3d new_pose = sampleMultivariateDistribution(mu, sigma);
      // std::cout << "new pose" << std::endl;
      // std::cout << new_pose << std::endl;

      // update previous pose of particle
      particle.prev_pose = particle.pose;
      particle.pose = new_pose;


      // double pose_likelihood = poseLikelihoodOdom(particle.pose, particle.prev_pose, cur_od, prev_od);
      // // double pose_likelihood = poseLikelihoodTwist(particle.pose, particle.prev_pose, u);
      // std::cout << "pose_likelihood" << std::endl;
      // std::cout << pose_likelihood << std::endl;


      // update weights
      // TODO: should I multply here?
      particle.weight *= eta;

    }


    // TODO: update map here
    Vector2D v(particle.pose(1), particle.pose(2));
    Transform2D T_pose(v, particle.pose(0));
    particle.grid.integrateScan(scan, T_pose);


    // const auto scan_likelihood = particle.grid.likelihoodFieldModel(scan, T_pose);
    // std::cout << "scan_likelihood" << std::endl;
    // std::cout << scan_likelihood << std::endl;


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
  Vector3d pose = particle_set_.at(idx).pose;

  Vector2D vec(pose(1), pose(2));
  Transform2D T_pose(vec, pose(0));

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



void ParticleFilter::sampleMotionModel(const Twist2D &u, Ref<Vector3d> pose)
{
  // sample noise
  VectorXd w = sampleMultivariateDistribution(motion_noise_);

  // update robot pose based on odometry
  if (almost_equal(u.w, 0.0))
  {
    // update theta
    pose(0) = normalize_angle_PI(pose(0) + w(0));
    // update x
    pose(1) += u.vx * std::cos(pose(0)) + w(1);
    // update y
    pose(2) += u.vx * std::sin(pose(0)) + w(2);
  }

  else
  {
    // update theta
    pose(0) = normalize_angle_PI(pose(0) + u.w + w(0));
    // update x
    pose(1) += (-u.vx / u.w) * std::sin(pose(0)) + \
                            (u.vx / u.w) * std::sin(pose(0) + u.w) + w(1);
    // update y
    pose(2) += (u.vx / u.w) * std::cos(pose(0)) - \
                            (u.vx / u.w) * std::cos(pose(0) + u.w) + w(2);
  }
}



double ParticleFilter::poseLikelihoodTwist(const Ref<Vector3d> cur_pose, const Ref<Vector3d> prev_pose, const Twist2D &u)
{
  const auto a1 = 0.1;
  const auto a2 = 0.2;
  const auto a3 = 0.1;
  const auto a4 = 0.2;
  const auto a5 = 0.1;
  const auto a6 = 0.2;

  const auto num = (prev_pose(1) - cur_pose(1))*std::cos(prev_pose(0)) + (prev_pose(2) - cur_pose(2))*std::sin(prev_pose(0));
  const auto denom = (prev_pose(2) - cur_pose(2))*std::cos(prev_pose(0)) + (prev_pose(1) - cur_pose(1))*std::sin(prev_pose(0));

  if (almost_equal(denom, 0.0))
  {
    throw std::invalid_argument("denom is 0 in poseLikelihood");
  }

  const auto frac = 0.5 * (num  / denom);

  const auto xstar = 0.5 * (prev_pose(1) + cur_pose(1)) + frac * (prev_pose(2) - cur_pose(2));
  const auto ystar = 0.5 * (prev_pose(2) + cur_pose(2)) + frac * (cur_pose(1) - prev_pose(1));

  const auto rstar = std::sqrt(std::pow((prev_pose(1) - xstar), 2) + std::pow((prev_pose(2) - ystar), 2));
  const auto dtheta = std::atan2(cur_pose(2) - ystar, cur_pose(1) - xstar) - std::atan2(prev_pose(2) - ystar, prev_pose(1) - xstar);

  const auto v_hat = dtheta * rstar * 1.0 / static_cast<double> (frequency_);
  const auto w_hat = dtheta * 1.0 / static_cast<double> (frequency_);
  const auto y_hat = (cur_pose(0) - prev_pose(0)) * 1.0 / static_cast<double> (frequency_) - w_hat;

  auto var1 = a1*u.vx*u.vx + a2*u.w*u.w;
  auto var2 = a3*u.vx*u.vx + a4*u.w*u.w;
  auto var3 = a5*u.vx*u.vx + a6*u.w*u.w;

  if (almost_equal(var1, 0.0))
  {
    var1 = a1 + a2;
  }

  if (almost_equal(var2, 0.0))
  {
    var2 = a3 + a4;
  }

  if (almost_equal(var3, 0.0))
  {
    var3 = a5 + a6;
  }

  const auto p1 = pdfNormal(u.vx - v_hat, var1);
  const auto p2 = pdfNormal(u.w - w_hat, var2);
  const auto p3 = pdfNormal(y_hat, var2);

  return p1*p2*p3;
}



double ParticleFilter::poseLikelihoodOdom(const Ref<Vector3d> cur_pose, const Ref<Vector3d> prev_pose,
                                          const Ref<Vector3d> cur_odom, const Ref<Vector3d> prev_odom)
{

  // // sample noise
  // VectorXd w = sampleMultivariateDistribution(motion_noise_);
  //
  //
  // // difference in odometry
  // const auto dx_odom = cur_odom(1) - prev_odom(1) + w(1);
  // const auto dy_odom = cur_odom(2) - prev_odom(2) + w(2);
  // const auto dth_odom = normalize_angle_PI(normalize_angle_PI(cur_odom(0)) - \
  //                         normalize_angle_PI(prev_odom(0)) + w(0));
  //
  //
  // // difference in state estimate plus noise
  // const auto dx_est = cur_pose(1) - prev_pose(1);// + w(1);
  // const auto dy_est = cur_pose(2) - prev_pose(2);// + w(2);
  // const auto dth_est = normalize_angle_PI(normalize_angle_PI(cur_pose(0)) - \
  //                            normalize_angle_PI(prev_pose(0)) /*+ w(0)*/);
  //
  //
  // // std::cout << dx_odom*dx_odom << std::endl;
  // // std::cout << dy_odom*dy_odom << std::endl;
  // // std::cout << dth_odom*dth_odom<< std::endl;
  //
  //
  // // compose probabilities
  // const auto p1 = pdfNormal(dx_est, dx_odom*dx_odom);
  // const auto p2 = pdfNormal(dy_est, dy_odom*dy_odom);
  // const auto p3 = pdfNormal(dth_est, dth_odom*dth_odom);
  // return p1 * p2 *p3;


  // Table 5.5 Probabilistic Robotics

  const auto a1_ = 0.1;
  const auto a2_ = 0.2;
  const auto a3_ = 0.1;
  const auto a4_ = 0.2;


  // difference between odometry measurements
  const auto rot1 = std::atan2(cur_odom(2) - prev_odom(2),
                               cur_odom(1) - prev_odom(1)) - prev_odom(0);

  const auto trans = std::sqrt(std::pow(cur_odom(1) - prev_odom(1), 2)
                                + std::pow(cur_odom(2) - prev_odom(2), 2));

  const auto rot2 = normalize_angle_PI(normalize_angle_PI(cur_odom(0)) - \
                          normalize_angle_PI(prev_odom(0)) - rot1);




  // difference beween pose estimates
  const auto rot1_hat = std::atan2(cur_pose(2) - prev_pose(2),
                                   cur_pose(1) - prev_pose(1)) - prev_pose(0);

  const auto trans_hat = std::sqrt(std::pow(cur_pose(1) - prev_pose(1), 2)
                                + std::pow(cur_pose(2) - prev_pose(2), 2));

  const auto rot2_hat = normalize_angle_PI(normalize_angle_PI(cur_pose(0)) - \
                            normalize_angle_PI(prev_pose(0)) - rot1_hat);



  // temporary variables for composing probabilities
  const auto temp1 = a1_*rot1_hat*rot1_hat + a2_*trans_hat*trans_hat;

  const auto temp2 = a3_*trans_hat*trans_hat
                     + a4_*rot1_hat*rot1_hat
                     + a4_*rot2_hat*rot2_hat;

  const auto temp3 = a1_*rot2_hat*rot2_hat + a2_*trans_hat*trans_hat;


  // std::cout << "temp1: " << temp1 << std::endl;
  // std::cout << "temp2: " << temp2 << std::endl;
  // std::cout << "temp3: " << temp3 << std::endl;



  // compose probabilities
  const auto p1 = pdfNormal(normalize_angle_PI(normalize_angle_PI(rot1) - normalize_angle_PI(rot1_hat)), temp1);

  const auto p2 = pdfNormal(trans - trans_hat, temp2);

  const auto p3 = pdfNormal(normalize_angle_PI(normalize_angle_PI(rot2) - normalize_angle_PI(rot2_hat)), temp3);

  return p1 * p2 *p3;
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
     // const auto U = r + static_cast<double> (m * (1.0 / num_particles_));
     const auto U = r + static_cast<double> (m * (1.0 / (num_particles_ - 1)));
     while(U > c)
     {
       i++;
       if (i > num_particles_ - 1)
       {
         i = num_particles_ - 1;
         // std::cout << "i to large: " << i << " size: " << particle_set_.size() << std::endl;
         break;
       }
       c += particle_set_.at(i).weight;
     }
     temp_particle_set.push_back(particle_set_.at(i));
  }

  particle_set_.clear();
  particle_set_ = temp_particle_set;
}



void ParticleFilter::sampleMode(const Transform2D &T, std::vector<Vector3d> &sampled_poses)
{
  TransformData2D T2d = T.displacement();
  Vector3d mu(T2d.theta, T2d.x, T2d.y);

  for(auto i = 0; i < k_; i++)
  {
    Vector3d sample = sampleMultivariateDistribution(mu, sample_range_);
    sample(0) = normalize_angle_PI(sample(0));
    sampled_poses.push_back(sample);

    // std::cout << "sample" << std::endl;
    // std::cout << sample << std::endl;

  }
}


void ParticleFilter::gaussianProposal(std::vector<Vector3d> &sampled_poses,
                                        Particle &particle,
                                        const Twist2D &u,
                                        const std::vector<float> &scan,
                                        const Ref<Vector3d> cur_odom,
                                        const Ref<Vector3d> prev_odom,
                                        Ref<Vector3d> mu,
                                        Ref<MatrixXd> sigma,
                                        double &eta)
{

  std::vector<double> likelihoods(k_);

  for(auto i = 0; i < k_; i++)
  {
    Vector3d xj = sampled_poses.at(i);
    Vector2D vec(xj(1), xj(2));
    Transform2D Txj(vec, xj(0));


    auto p_scan = particle.grid.likelihoodFieldModel(scan, Txj);
    auto p_pose = poseLikelihoodOdom(xj, particle.prev_pose, cur_odom, prev_odom);
    // auto p_pose = poseLikelihoodTwist(xj, particle.prev_pose, u);


    p_scan = std::clamp(p_scan, 1.0, 20.0);
    p_pose = std::clamp(p_pose, 1.0, 20.0);



    const auto p = p_scan * p_pose;
    // const auto p = p_pose;

    // std::cout << "p_scan" << std::endl;
    // std::cout << p_scan << std::endl;
    //
    // std::cout << "p_pose" << std::endl;
    // std::cout << p_pose << std::endl;
    // //
    // std::cout << "p" << std::endl;
    // std::cout << p << std::endl;


    likelihoods.at(i) = p;

    mu += xj * p;
    eta += p;
  }


  if (almost_equal(eta, 0.0))
  {
    throw std::invalid_argument("eta is 0");
  }


  // scale mu
  mu /= eta;
  mu(0) = normalize_angle_PI(mu(0));


  for(auto i = 0; i < k_; i++)
  {
    Vector3d xj = sampled_poses.at(i);
    Vector2D vec(xj(1), xj(2));
    Transform2D Txj(vec, xj(0));

    sigma += (xj - mu) * (xj - mu).transpose() * likelihoods.at(i);
  }

  // scale sigma
  sigma /= eta;
}


Transform2D ParticleFilter::icpInitGuess(const Ref<Vector3d> cur_odom, const Ref<Vector3d> prev_odom)
{
  const auto dx = cur_odom(1) - prev_odom(1);
  const auto dy = cur_odom(2) - prev_odom(2);
  const auto dth = normalize_angle_PI(normalize_angle_PI(cur_odom(0)) - \
                                          normalize_angle_PI(prev_odom(0)));

  Vector2D v(dx, dy);
  Transform2D T(v, dth);
  return T;
}


} // end namespace

// end file



// // find transform from previous pose to current pose
// // map to current pose
// Vector2D cur_v(cur_pose(1), cur_pose(2));
// Transform2D Tmx_prime(cur_v, cur_pose(0));
//
// // map to previous pose
// Vector2D prev_v(prev_pose(1), prev_pose(2));
// Transform2D Tmx(prev_v, prev_pose(0));
//
// // previous pose to current pose
// Transform2D Txx_prime = Tmx.inv() * Tmx_prime;
//
//
// // at this point assume dt = 1
// // vb = T^-1 * T_dot
// Transform2D Tvb = Txx_prime.inv() * Txx_prime;
// TransformData2D T2dvb = Tvb.displacement();
//
// // scale by loop frequency
// Twist2D vb;
// vb.w = T2dvb.theta * 1.0 / static_cast<double> (frequency_);
// vb.vx = T2dvb.x * 1.0 / static_cast<double> (frequency_);
// vb.vy = T2dvb.y * 1.0 / static_cast<double> (frequency_); // should be 0
//
//
// std::cout << "Predicted twist" << std::endl;
// std::cout << vb << std::endl;
//
// std::cout << "Odom twist" << std::endl;
// std::cout << u << std::endl;
//
//
// auto var = a1*u.vx*u.vx + a2*u.w*u.w;
// if (almost_equal(var, 0.0))
// {
//   var = a1 + a2;
// }
