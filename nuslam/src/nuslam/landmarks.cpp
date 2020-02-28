/// \file
/// \brief

#include <iostream>


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


double pointDistance(const Vector2D &v1, const Vector2D &v2)
{
  return std::sqrt(std::pow(v2.x - v1.x, 2) + std::pow(v2.y - v1.y, 2));
}






Landmarks::Landmarks(const LaserProperties &props, double epsilon)
                          : beam_min(props.beam_min),
                            beam_max(props.beam_max),
                            beam_delta(props.beam_delta),
                            range_min(props.range_min),
                            range_max(props.range_max),
                            epsilon(epsilon)
{
}


void Landmarks::laserEndPoints(std::vector<Vector2D> &end_points,
                                  const std::vector<float> &beam_length)
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
  // clear current landmarks
  lm.clear();

  // store points in temp vector
  std::vector<Vector2D> temp_points;

  // compare previous and current points
  Vector2D curr_point, prev_point;
  curr_point = end_points.at(0);
  prev_point = curr_point;

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
      // std::cout << temp_points.size() << std::endl;

      // check if temp as less than three points
      if (temp_points.size() >= 3)
      {
        // std::cout << "new cluster" << std::endl;

        // create new cluster
        // and add cluster to landmarks
        Cluster cluster(temp_points);
        lm.push_back(cluster);
      }

      // clear temp list
      temp_points.clear();
    }

    prev_point = curr_point;

  } // end loop

  // std::cout << temp_points.size() << std::endl;




  // compare last point to first point
  // check if they are within the same cluster
  // const auto dist = pointDistance(end_points.front(), end_points.back());
  //
  // if (dist <= epsilon)
  // {
  //   lm.front().points.insert(lm.front().points.end(),
  //                            lm.back().points.begin(),
  //                            lm.back().points.end());
  //
  //   lm.pop_back();
  // }

}



bool Landmarks::generateClusters(const std::vector<float> &beam_length)
{
  std::vector<Vector2D> end_points;
  laserEndPoints(end_points, beam_length);

  clusterScan(end_points);

  return lm.empty() ? false : true;
}








} // end namespace
























// end file
