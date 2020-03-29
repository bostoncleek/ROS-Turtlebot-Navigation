/// \file
/// \brief

#include <iostream>
#include <functional>
#include <numeric>
#include <iomanip>
#include <Eigen/Dense>

#include "nuslam/landmarks.hpp"


namespace nuslam
{


Vector2D range2Cartesian(double range, double beam_angle)
{
  Vector2D pt;
  pt.x = range * std::cos(beam_angle);
  pt.y = range * std::sin(beam_angle);

  return pt;
}


double pointDistance(const Vector2D p1, const Vector2D p2)
{
  const auto dx = p1.x - p2.x;
  const auto dy = p1.y - p2.y;

  return std::sqrt(dx*dx + dy*dy);
}


double lawCosines(const double a, const double b, const double c)
{
  const auto var = (b*b - a*a - c*c) / (-2.0 * a * c);
  return std::acos(var);
}


void centroid(Cluster &cluster)
{

  // x center
  const auto x = std::accumulate(cluster.points.begin(), cluster.points.end(), 0.0, \
                            std::bind(std::plus<double>(), std::placeholders::_1,
                            std::bind(&Vector2D::x, std::placeholders::_2)));

  cluster.x_hat = x / cluster.points.size();


  // y center
  const auto y = std::accumulate(cluster.points.begin(), cluster.points.end(), 0.0, \
                            std::bind(std::plus<double>(), std::placeholders::_1,
                            std::bind(&Vector2D::y, std::placeholders::_2)));

  cluster.y_hat = y / cluster.points.size();
}



void shiftCentroidToOrigin(Cluster &cluster)
{
  for(auto &point : cluster.points)
  {
    point.x -= cluster.x_hat;
    point.y -= cluster.y_hat;

    const auto zi = point.x*point.x + point.y*point.y;

    cluster.z.push_back(zi);
  }

  const auto x_bar =  std::accumulate(cluster.points.begin(), cluster.points.end(), 0.0, \
                            std::bind(std::plus<double>(), std::placeholders::_1,
                            std::bind(&Vector2D::x, std::placeholders::_2)));

  cluster.x_bar = x_bar / static_cast<double> (cluster.points.size());



  const auto y_bar = std::accumulate(cluster.points.begin(), cluster.points.end(), 0.0, \
                            std::bind(std::plus<double>(), std::placeholders::_1,
                            std::bind(&Vector2D::y, std::placeholders::_2)));

  cluster.y_bar = y_bar / static_cast<double> (cluster.points.size());



  const auto z_bar =  std::accumulate(cluster.z.begin(), cluster.z.end(), 0.0);

  cluster.z_bar = z_bar / static_cast<double> (cluster.points.size());
}



void composeCircle(Cluster &cluster)
{
  // compose the z matrix
  Eigen::MatrixXd Z(cluster.points.size(), 4);

  for(unsigned int i = 0; i < cluster.points.size(); i++)
  {
    Z(i,0) = std::pow(cluster.points.at(i).x, 2) + std::pow(cluster.points.at(i).y, 2);
    Z(i,1) = cluster.points.at(i).x;
    Z(i,2) = cluster.points.at(i).y;
    Z(i,3) = 1.0;

    // std::cout << cluster.points.at(i).x << " " << cluster.points.at(i).y << std::endl;
  }


  // // compose the M matrix
  // Eigen::MatrixXd M(4,4);
  // M = Z.transpose() * Z;
  // M *= (1.0 / static_cast<double> (cluster.points.size()));
  //
  //
  // // Hyperaccurate algebraic fit
  // Eigen::MatrixXd H(4,4);
  // H << 8.0*cluster.z_bar, 0.0, 0.0, 2.0,
  //      0.0,               1.0, 0.0, 0.0,
  //      0.0,               0.0, 1.0, 0.0,
  //      2.0,               0.0, 0.0, 0.0;


  // H^-1
  Eigen::MatrixXd Hinv(4,4);
  Hinv << 0.0, 0.0, 0.0, 0.5,
          0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0,
          0.5, 0.0, 0.0, -2.0*cluster.z_bar;


  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Z, Eigen::ComputeFullU |
                                        Eigen::ComputeFullV);



  Eigen::MatrixXd Sigma = Eigen::MatrixXd::Zero(4,4);
  Sigma(0,0) = svd.singularValues()(0);
  Sigma(1,1) = svd.singularValues()(1);
  Sigma(2,2) = svd.singularValues()(2);
  Sigma(3,3) = svd.singularValues()(3);

  // std::cout << "Sigma: " << std::endl;
  // std::cout << Sigma << std::endl;
  // std::cout << Sigma.rows() << std::endl;
  // std::cout << Sigma.cols() << std::endl;


  // take the smallest singular value
  const auto sigma4 = svd.singularValues()(3);
  // std::cout << "smallest singular value: " << sigma4 << std::endl;


  Eigen::MatrixXd A(4,1);

  Eigen::MatrixXd V = svd.matrixV();
  // std::cout << "V: " << std::endl;
  // std::cout << V << std::endl;


  if (sigma4 < 1e-12)
  {
    // std::cout << "sigma4 < 1e-12" << std::endl;
    A = V.col(3);
  }


  else
  {
    // std::cout << "sigma4 > 1e-12" << std::endl;
    // std::cout.precision(10);

    Eigen::MatrixXd Y = V * Sigma * V.transpose();
    Eigen::MatrixXd Q = Y * Hinv * Y;

    // std::cout << "Y: " << std::endl;
    // std::cout << Y << std::endl;

    // find index of smallest
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(Q);
    Eigen::MatrixXd values(es.eigenvalues());

    // std::cout << "Eigen vectors: " << std::endl;
    // std::cout << es.eigenvectors() << std::endl;
    //
    // std::cout << "Eigen values: " << std::endl;
    // std::cout << values << std::endl;

    int idx = 0;
    auto temp = 0.0;
    auto smallest = 1e12;
    for(int i = 0; i < values.rows(); i++)
    {
      temp = values(i);
      if (temp > 0.0 and temp < smallest)
      {
        smallest = temp;
        idx = i;
      }
    }

    // std::cout << "Index: " << idx << std::endl;

    Eigen::MatrixXd Astar(es.eigenvectors().col(idx));

    // std::cout << "A*: " << std::endl;
    // std::cout << Astar << std::endl;

    A = Y.completeOrthogonalDecomposition().solve(Astar);
  }


  // std::cout << "A: " << std::endl;
  // std::cout << A << std::endl;


  const auto a = -A(1) / (2.0 * A(0));
  const auto b = -A(2) / (2.0 * A(0));

  const auto R2 = (A(1)*A(1) + A(2)*A(2) - 4.0*A(0)*A(3)) / (4.0*A(0)*A(0));


  // std::cout << a << " " << b << " " << R2 << std::endl;


  // update the actual center
  cluster.x_hat += a;
  cluster.y_hat += b;

  cluster.radius = std::sqrt(R2);

}




Landmarks::Landmarks(const LaserProperties &props, double epsilon)
                          : beam_min(props.beam_min),
                            beam_max(props.beam_max),
                            beam_delta(props.beam_delta),
                            range_min(props.range_min),
                            range_max(props.range_max),
                            epsilon(epsilon),
                            radius_thresh(0.05),
                            angle_std(0.15),
                            mu_min(90.0),
                            mux_max(135.0),
                            num_points(4)
{
}



void Landmarks::featureDetection(const std::vector<float> &beam_length)
{
  // convert scan to cartesian coordinates
  std::vector<Vector2D> end_points;
  laserEndPoints(end_points, beam_length);

  // cluster the points
  clusterScan(end_points);


  // fit circles
  for(unsigned int i = 0; i < lm.size(); i++)
  {
    centroid(lm.at(i));
    shiftCentroidToOrigin(lm.at(i));
    composeCircle(lm.at(i));
  }


  // classify the circles
  for(unsigned int i = 0; i < lm.size(); i++)
  {
    // // // remove it if not a circle
    // if (!classifyCircles(lm.at(i)) or lm.at(i).radius > radius_thresh)
    // {
    //   // std::cout << "Not a circle" << std::endl;
    //   lm.erase(lm.begin() + i);
    //   // decrement i because if something is deleted
    //   // the size of the landmark vector changes
    //   i--;
    // }

    // check if any circles are left
    // if (lm.empty())
    // {
    //   std::cout << "No circles found" << std::endl;
    //   return;
    // }

    // // eliminate circles with large radius
    if (lm.at(i).radius > radius_thresh)
    {
      // std::cout << "radius: " << lm.at(i).radius << std::endl;
      lm.erase(lm.begin() + i);

      // decrement i because if something is deleted
      // the size of the landmark vector changes
      i--;
    }
  }

}



void Landmarks::laserEndPoints(std::vector<Vector2D> &end_points,
                                  const std::vector<float> &beam_length) const
{

  auto beam_angle = beam_min;
  auto range = 0.0;


  for(unsigned int i = 0; i < beam_length.size(); i++)
  {
    range = beam_length.at(i);

    if (range >= range_min and range < range_max)
    {
      Vector2D point = range2Cartesian(range, beam_angle);
      end_points.push_back(point);
    }


    // update beam angle
    beam_angle += beam_delta;

    // max angle is negative
    if (beam_max < 0 and beam_angle < beam_max)
    {
      beam_angle = beam_min;
    }

    // max angle is positive
    else if (beam_max >= 0 and beam_angle > beam_max)
    {
      beam_angle = beam_min;
    }

  } // end loop

}



void Landmarks::clusterScan(const std::vector<Vector2D> &end_points)
{
  // TODO: fixed the edge case where the scan starts in the center of
  //        a cluster. This is was causing a segfault during testing.


  // clear current landmarks
  lm.clear();

  // store points in temp vector
  std::vector<Vector2D> temp_points;

  // compare previous and current points
  Vector2D curr_point, prev_point;
  curr_point = end_points.at(0);
  prev_point = curr_point;

  // std::cout << "----------------" << std::endl;
  // std::cout << "size: " << end_points.size() << std::endl;


  for(unsigned int i = 0; i < end_points.size(); i++)
  {
    curr_point = end_points.at(i);

    const auto dist = pointDistance(curr_point, prev_point);

    // std::cout << "----------------" << std::endl;
    // std::cout << curr_point;
    // std::cout << prev_point;
    // std::cout << dist << std::endl;
    // std::cout << "----------------" << std::endl;

    if (dist <= epsilon)
    {
      // std::cout << "in cluster" << std::endl;
      temp_points.push_back(end_points.at(i));
    }


    else if (dist > epsilon)
    {
      // std::cout << "new cluster" << std::endl;
      // create new cluster
      // and add cluster to landmarks
      Cluster cluster(temp_points);
      lm.push_back(cluster);

      // clear temp list
      temp_points.clear();
    }

    prev_point = curr_point;

  } // end loop

  // add last cluster
  // std::cout << "new cluster" << std::endl;
  Cluster cluster(temp_points);
  lm.push_back(cluster);

  // std::cout << "----------------" << std::endl;


  // compare last point to first point
  // check if they are within the same cluster
  if (lm.size() > 1)
  {
    const auto dist = pointDistance(end_points.front(), end_points.back());

    if (dist <= epsilon)
    {
      // std::cout << "scan in center of cluster" << std::endl;
      lm.front().points.insert(lm.front().points.end(),
                               lm.back().points.begin(),
                               lm.back().points.end());

      lm.pop_back();
    }
  }


  // remove clusters less than four points
  for(unsigned int i = 0; i < lm.size(); i++)
  {
    if (lm.at(i).points.size() < num_points)
    {
      lm.erase(lm.begin() + i);
      i--;
    }
  }

}

bool Landmarks::classifyCircles(const Cluster &cluster) const
{
  const Vector2D p_start = cluster.points.front();
  const Vector2D p_end = cluster.points.back();

  // p_start to p_end
  const auto b = pointDistance(p_start, p_end);

  // number of points between the endpoints
  unsigned int num_inner_points = cluster.points.size()-2;

  // store angles in vector
  std::vector<double> angles;
  angles.reserve(num_inner_points);


  // loop through all points between the endpoints
  for(unsigned int i = 1; i < cluster.points.size()-1; i++)
  {
    const Vector2D p = cluster.points.at(i);

    // p_start to p
    const auto c = pointDistance(p_start, p);

    // p to p_end
    const auto a = pointDistance(p, p_end);

    // angle
    angles.push_back(lawCosines(a, b, c));
  }


  // average angle
  auto mean_angle = std::accumulate(angles.begin(), angles.end(), 0.0);
  mean_angle /= static_cast<double>(num_inner_points);

  // standard deviation of angles
  auto sum = 0.0;
  for(const auto angle: angles)
  {
    const auto delta = angle - mean_angle;
    sum += delta*delta;
  }

  const auto sigma = std::sqrt(sum / static_cast<double>(num_inner_points));


  // std::cout << "mean angle: " << rigid2d::rad2deg(mean_angle) << std::endl;
  // std::cout << "sigma: " << sigma << std::endl;



  // is a circle
  if ((sigma < angle_std) and \
      (mean_angle <= rigid2d::deg2rad(mux_max)) and \
      (mean_angle >= rigid2d::deg2rad(mu_min)))
  {
    return true;
  }

  return false;
}


} // end namespace

// end file
