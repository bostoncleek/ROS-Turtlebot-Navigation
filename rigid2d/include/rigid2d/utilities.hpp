#ifndef UTILITIES_HPP
#define UTILITIES_HPP
/// \file
/// \brief utility functions

#include <random>
#include <Eigen/Dense>



namespace rigid2d
{

using Eigen::MatrixXd;
using Eigen::VectorXd;

/// \brief Returns a random number engine
std::mt19937_64 &getTwister();

/// \brief Samples a normal distribution with a specified mean and variance
/// \param mu - mean of distribution
/// \param sigma - variance of distribution
/// \returns a random sample
double sampleNormalDistribution(const double mu, const double sigma);

/// \brief samples a standard normal distribution
/// \param n - number of samples
/// \returns - random sample
VectorXd sampleStandardNormal(int n);

/// \brief samples a multivariate standard normal distribution
/// \param cov - covariance noise matrix
/// \returns - random samples
VectorXd sampleMultivariateDistribution(MatrixXd cov);




}

#endif
