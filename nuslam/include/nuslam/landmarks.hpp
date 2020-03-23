#ifndef LANDMARKS_GUARD_HPP
#define LANDMARKS_GUARD_HPP
/// \file
/// \brief Feature detector classifies circles

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
  /// \param p1 - point 1
  /// \param p2 - point 2
  /// returns distance between p1 and p2
  double pointDistance(const Vector2D p1, const Vector2D p2);

  /// \brief Compose angle in triangle using law of cosines
  /// \param a - length of side 1
  /// \param b - length of side 2
  /// \param c - length of side 3
  /// reutrns - angle between sides a (1) and c (3)
  double lawCosines(const double a, const double b, const double c);


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
    std::vector<double> z;

    double radius = 0.0;            // radius of cluster
    double x_hat = 0.0;                 // x - centroid
    double y_hat = 0.0;                 // y - centroid

    double x_bar = 0.0;
    double y_bar = 0.0;
    double z_bar = 0.0;

    /// \brief Default cluster constructor
    Cluster() {}

    /// \brief Default cluster constructor
    /// \param points - 2d points
    Cluster(const std::vector<Vector2D> &points) : points(points) {}
  };


  /// \brief landmark detection and classification
  class Landmarks
  {
  public:
    /// \brief Costructs feature detector
    /// \param props - properties of the laser scanner
    /// \param epsilon - distance thrshold between points
    Landmarks(const LaserProperties &props, double epsilon);

    /// \brief Detect circles in point cloud
    /// \param beam_length - laser scan
    void featureDetection(const std::vector<float> &beam_length);

    // list of landmarks
    std::vector<Cluster> lm;

  private:
    /// \brief Convertes range and bearing to cartesian coordinates
    /// \param end_points - 2D point cloud
    /// \param beam_length - laser scan
    void laserEndPoints(std::vector<Vector2D> &end_points,
                       const std::vector<float> &beam_length);

    /// \brief Groups the laser scan into clusters
    /// \param end_points - 2D point cloud
    void clusterScan(const std::vector<Vector2D> &end_points);

    /// \brief Compose centroid of cluster
    /// \param cluster - group of candidate points
    void centroid(Cluster &cluster);

    /// \brief shifts the centorid of the cluster to the center
    /// \param cluster - group of candidate points
    void shiftCentroidToOrigin(Cluster &cluster);

    /// \brief Fite a circle to the cluster
    /// \param cluster - group of candidate points
    void composeCircle(Cluster &cluster);

    /// \brief Classifies a cluster as a circle or not
    /// \param cluster - group of candidate points
    bool classifyCircles(const Cluster &cluster);


    double beam_min, beam_max, beam_delta;    // beam angles
    double range_min, range_max;              // range limits
    double epsilon;                           // distance threshold for clustering
    double radius_thresh;                     // threshold for radius size

    double angle_std;                         // standard deviation of angles
    double mu_min, mux_max;                   // min and max mean angles of circle
    unsigned int num_points;                  // min points per circle
  };
}







#endif
