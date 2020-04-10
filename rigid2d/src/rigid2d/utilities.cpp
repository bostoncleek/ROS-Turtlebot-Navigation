// \file
/// \brief utility function implementations


#include "rigid2d/utilities.hpp"



namespace rigid2d
{

std::mt19937_64 &getTwister()
{
  static std::random_device rd;
  static std::mt19937_64 gen(rd());
  return gen;
}


double sampleNormalDistribution(const double mu, const double sigma)
{
  std::normal_distribution<double> dis(mu, sigma);
  return dis(getTwister());
}


double sampleUniformDistribution(const double min, const double max)
{
  std::uniform_real_distribution<double> dis(min, max);
  return dis(getTwister());
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


VectorXd sampleMultivariateDistribution(MatrixXd cov)
{
  // must be square
  int dim = cov.cols();
  VectorXd rand_vec = sampleStandardNormal(dim);

  // cholesky decomposition
  MatrixXd L( cov.llt().matrixL() );

  return L * rand_vec;
}


double euclideanDistance(double x0, double y0, double x1, double y1)
{
  const auto dx = x0 - x1;
  const auto dy = y0 - y1;
  return std::sqrt(dx*dx + dy*dy);
}


}
