#ifndef LANDMARKS_GUARD_HPP
#define LANDMARKS_GUARD_HPP
/// \file
/// \brief

#include <cmath>
#include <iosfwd>
#include <vector>


#include <rigid2d/rigid2d.hpp>

namespace nuslam
{
  using rigid2d::Vector2D;

  /// \brief convert laser range measurements to cartesian coordinates
  /// \param range - range measurement
  /// \param bean_angle - angle of lidar beam
  Vector2D range2Cartesian(double range, double beam_angle);

  /// \brief distance between two points
  /// \param v1 - point 1
  /// \param v2 - point 2
  double pointDistance(const Vector2D &v1, const Vector2D &v2);



  /// \brief stores laser range finder limits and properties
  struct LaserProperties
  {
    double beam_min;
    double beam_max;
    double beam_delta;
    double range_min;
    double range_max;

  /// \brief Set all properties to specification
  /// \param beam_min - start angle of scan
  /// \param beam_max - end angle of scan
  /// \param beam_delta - increment scan angle
  /// \param range_min - min range limit for laser
  /// \param range_max - max range limit for laser
  LaserProperties(double beam_min, double beam_max, double beam_delta,
                  double range_min, double range_max)
                      : beam_min(beam_min),
                        beam_max(beam_max),
                        beam_delta(beam_delta),
                        range_min(range_min),
                        range_max(range_max) {}
  };


  /// \brief stores each landmark
  struct Cluster
  {
    std::vector<Vector2D> points;   // point in the landmark
    double radius = 0.0;            // radius of cluster
    double x = 0.0;                 // x - centorid
    double y = 0.0;                 // y - centroid

    /// \brief
    Cluster(const std::vector<Vector2D> &points) : points(points) {}
  };



  /// \brief landmark detection and classification
  class Landmarks
  {
  public:
    Landmarks(const LaserProperties &props, double epsilon);


    void laserEndPoints(std::vector<Vector2D> &end_points,
                       const std::vector<float> &beam_length);


    void clusterScan(const std::vector<Vector2D> &end_points);


    bool generateClusters(const std::vector<float> &beam_length);


    // list of landmarks
    std::vector<Cluster> lm;

  private:
    double beam_min, beam_max, beam_delta;
    double range_min, range_max;
    double epsilon;
  };
}







#endif
