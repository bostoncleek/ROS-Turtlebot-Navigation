/// \file
/// \brief models for a 2D laser range finder

#include "bmapping/sensor_model.hpp"

namespace bmapping
{

Vector2D range2Cartesian(double range, double beam_angle)
{
  Vector2D pt;
  pt.x = range * std::cos(beam_angle);
  pt.y = range * std::sin(beam_angle);

  return pt;
}




// Class LaserScanner


LaserScanner::LaserScanner(const LaserProperties &props, const Transform2D &Trs)
                          : z_hit_(props.z_hit),
                            z_short_(props.z_short),
                            z_max_(props.z_max),
                            z_rand_(props.z_rand),
                            sigma_hit_(props.sigma_hit),
                            Trs_(Trs),
                            beam_min_(props.beam_min),
                            beam_max_(props.beam_max),
                            beam_delta_(props.beam_delta),
                            range_min_(props.range_min),
                            range_max_(props.range_max)

{
}




void LaserScanner::laserEndPoints(std::vector<Vector2D> &end_points,
                                  const std::vector<float> &beam_length,
                                  const Transform2D &pose) const
{

  // TODO: Implementation assumes map is aligned with world
  //       This may cause a bug because laser end points are suppose to
  //        be in frame of map

  // TODO: not sure if checking for valid measurements
  //       to reserve memory for vector reduces latency


  // number of measurements within specified range limits
  unsigned int num_meas = numberValidMeasurements(beam_length);

  // store end points
  end_points.reserve(num_meas);

  // end point
  Vector2D point;

  // TODO: Tmr = Twm^-1 * Twr then pm = Tmr * Trs * p
  // transform from map to sensor
  // Tms = Twr * Trs
  Transform2D Tms = pose * Trs_;


  // start beam angle at 0 and then transoform into coordinates
  // of robot and the to world
  double beam_angle = beam_min_;

  // range measurement
  double range = 0.0;
  for(unsigned int i = 0; i < beam_length.size(); i++)
  {
    range = beam_length.at(i);

    if (range >= range_min_ and range < range_max_)
    {
      // std::cout << range_min_ << " <= " << range << " < " << range_max_ << std::endl;
      // cartesian coordinates in frame of sensor
      point = range2Cartesian(range, beam_angle);

      // cartesian coordinates in frame of map
      // pm = Tms * p
      point = Tms(point);

      end_points.push_back(point);
    }


    // update beam angle
    beam_angle += beam_delta_;

    // max angle is negative
    if (beam_max_ < 0.0 and beam_angle <= beam_max_)
    {
      beam_angle = beam_min_;
    }

    // max angle is positive
    else if (beam_max_ >= 0.0 and beam_angle >= beam_max_)
    {
      beam_angle = beam_min_;
    }

  } // end loop

}



unsigned int LaserScanner::numberValidMeasurements(const std::vector<float> &beam_length) const
{
  // number of valid measurements
  unsigned int valid_meas = 0;


  // iterate and find number of valid measurements
  std::vector<float>::const_iterator it;
  for(it = beam_length.begin(); it != beam_length.end(); ++it)
  {
    if (*it >= range_min_ and *it < range_max_)
      valid_meas++;
  }

  return valid_meas;
}
} // end namespace
