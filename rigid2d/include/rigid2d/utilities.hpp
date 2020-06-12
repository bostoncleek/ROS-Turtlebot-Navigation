#ifndef UTILITIES_HPP
#define UTILITIES_HPP
/// \file
/// \brief utility functions

#include <random>
#include <eigen3/Eigen/Dense>



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

/// \brief Samples a uniform real distribution
/// \param min - lower bound
/// \param max - upper bound
/// \returns a random sample
double sampleUniformDistribution(const double min, const double max);

/// \brief samples a standard normal distribution
/// \param n - number of samples
/// \returns - random sample
VectorXd sampleStandardNormal(int n);

/// \brief samples a multivariate standard normal distribution
/// \param cov - covariance noise matrix
/// \returns - random samples
VectorXd sampleMultivariateDistribution(MatrixXd cov);


/// \brief Euclidean distance between two points
/// \param x0 - x position of first point
/// \param y0 - y position of first point
/// \param x1 - x position of second point
/// \param y1 - y position of second point
/// \return euclidean distance
double euclideanDistance(double x0, double y0, double x1, double y1);


}

#endif
