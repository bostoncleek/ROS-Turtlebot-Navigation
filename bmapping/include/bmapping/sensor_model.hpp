#ifndef SENSOR_MODEL_HPP
#define SENSOR_MODEL_HPP
/// \file
/// \brief models for a 2D laser range finder

#include <cmath>
#include <iosfwd>
#include <vector>

#include <rigid2d/rigid2d.hpp>


namespace bmapping
{
  using rigid2d::Vector2D;
  using rigid2d::Transform2D;


  /// \brief stores laser range finder limits and properties
  struct LaserProperties
  {
    // lidar beam
    float beam_min;
    float beam_max;
    float beam_delta;
    float range_min;
    float range_max;

    // Mixture params for the components of the model; must sum to 1
    double z_hit;
    double z_short;
    double z_max;
    double z_rand;

    // Stddev of Gaussian model for laser hits.
    double sigma_hit;


    /// \brief Set all properties to zero
    LaserProperties() : beam_min(0.0),
                        beam_max(0.0),
                        beam_delta(0.0),
                        range_min(0.0),
                        range_max(0.0),
                        z_hit(0.25),
                        z_short(0.25),
                        z_max(0.25),
                        z_rand(0.25),
                        sigma_hit(1) {}


    /// \brief Set all properties to specification
    /// \param beam_min - start angle of scan
    /// \param beam_max - end angle of scan
    /// \param beam_delta - increment scan angle
    /// \param range_min - min range limit for laser
    /// \param range_max - max range limit for laser
    /// \param z_hit -
    /// \param z_short -
    /// \param z_max - sensors max allowable value
    /// \param z_rand-
    /// \param sigma_hit - Stddev of Gaussian model for laser hits.
    LaserProperties(float beam_min, float beam_max, float beam_delta,
                    float range_min, float range_max, double z_hit,
                    double z_short, double z_max, double z_rand,
                    double sigma_hit)
                        : beam_min(beam_min),
                          beam_max(beam_max),
                          beam_delta(beam_delta),
                          range_min(range_min),
                          range_max(range_max),
                          z_hit(z_hit),
                          z_short(z_short),
                          z_max(z_max),
                          z_rand(z_rand),
                          sigma_hit(sigma_hit) {}


  };



  /// \brief Models a 2D laser range finder
  class LaserScanner
  {
  public:
    /// \brief creates a laser scanner
    /// \param props - contains properties about laser scan
    /// \parma Trs - transform from robot to laser scanner
    LaserScanner(const LaserProperties &props, const Transform2D &Trs);


    /// \brief compose cartesian end points of laser beams in map frame
    /// \param beam_length - range measurements from recent scan
    /// \param pose - robots pose in world (Twr)
    /// end_points[out] end points of laser beam
    void laserEndPoints(std::vector<Vector2D> &end_points,
                         const std::vector<float> &beam_length,
                         const Transform2D &pose) const;


    /// \brief Determines the number of range measurements with the
    ///        limits if the laser range finder
    /// \param beam_length - range measurements from recent scan
    /// \returns number of laser measurements with range limits of sensor
    unsigned int numberValidMeasurements(const std::vector<float> &beam_length) const;



    double z_hit_, z_short_, z_max_, z_rand_;  //must sum to 1
    double sigma_hit_;                         // Stddev of Gaussian model for laser hits

  private:
    Transform2D Trs_;                         // robot to laser scanner
    float beam_min_, beam_max_, beam_delta_;  // start, end, increment scan angles
    float range_min_, range_max_;              // min and max range limit for laser
  };

} // end namespace

#endif
