#ifndef PLANNER_UTILITIES_GUARD_HPP
#define PLANNER_UTILITIES_GUARD_HPP
// \file
/// \brief Utility methods for planning methods

#include <cmath>
#include <vector>

#include <rigid2d/rigid2d.hpp>
#include <rigid2d/utilities.hpp>


namespace planner
{
  using rigid2d::Vector2D;
  using rigid2d::euclideanDistance;

  typedef std::vector<Vector2D> polygon;      // convex polygon, vertices counter-clockwise
  typedef std::vector<polygon> obstacle_map;  // constain all polygons in Cspace

  /// \brief Useful information for determining the closest point to
  ///        a line segment
  struct ClosePoint
  {
    double t = 0.0;           // if t [0, 1] then the min distance is on the line segment
    double sign_d = 0.0;      // signed distance from line to p, d > 0 if on left side in direction p1 => p2
    Vector2D p;               // closest point to line
    bool on_seg = false;      // true if min distance to p is on the segment
  };


  /// \brief Compose shortest distance from linesegment to a point
  ///        p1 and p2 are the bounds of the line segment
  ///        and p3 is the point.
  ///        Checks if the min distance is on the line segment
  /// \param p1 - first bound of line segment
  /// \param p1 - second bound of line segment
  /// \param p3 - point to compose distance to
  /// distance[out] - min distance between line and a point
  /// \returns - true if the closest distance lies on the line segment
  bool minDistLineSegPt(double &distance,
                        const Vector2D &p1,
                        const Vector2D &p2,
                        const Vector2D &p3);

  /// \brief Compose shortest distance from linesegment to a point
  ///        p1 and p2 are the bounds of the line segment
  ///        and p3 is the point.
  ///        Does not check if the min distance is on the line segment
  /// \param p1 - first bound of line segment
  /// \param p1 - second bound of line segment
  /// \param p3 - point to compose distance to
  /// \returns - min distance between line and a point
  double minDistLineSegPt(const Vector2D &p1,
                          const Vector2D &p2,
                          const Vector2D &p3);

  /// \brief Compose shortest distance from linesegment to a point
  ///        p1 and p2 are the bounds of the line segment
  ///        and p3 is the point.
  ///        Checks if the min distance is on the line segment
  /// \param p1 - first bound of line segment
  /// \param p1 - second bound of line segment
  /// \param p3 - point to compose distance to
  /// \returns - data regarding the state of the min distance
  ///            from a point to the line segment
  ClosePoint signMinDist2Line(const Vector2D &p1,
                              const Vector2D &p2,
                              const Vector2D &p3);




}



#endif
