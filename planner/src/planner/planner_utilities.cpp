// \file
/// \brief Utility methods for planning methods

#include "planner/planner_utilities.hpp"

namespace planner
{

  bool minDistLineSegPt(double &distance,
                        const Vector2D &p1,
                        const Vector2D &p2,
                        const Vector2D &p3)
  {
    // source http://paulbourke.net/geometry/pointlineplane/

    const auto dx = p2.x - p1.x;
    const auto dy = p2.y - p1.y;

    const auto num = (p3.x - p1.x)*dx + (p3.y - p1.y)*dy;
    const auto denom = dx*dx + dy*dy;
    const auto u = num / denom;

    // std::cout << u << std::endl;

    // location of point (P) on line (not necessarily on the line segment)
    const auto px = p1.x + u * dx;
    const auto py = p1.y + u * dy;

    // min distance
    distance = euclideanDistance(px, py, p3.x, p3.y);

    if (rigid2d::almost_equal(u, 0.0) or rigid2d::almost_equal(u, 1.0))
    {
      return true;
    }

    else if (u > 0.0 and u < 1.0)
    {
      return true;
    }

    // return (u >= 0.0 and u <= 1.0) ? true : false;
    return false;
  }


  double minDistLineSegPt(const Vector2D &p1,
                          const Vector2D &p2,
                          const Vector2D &p3)
  {

    // vector from p1 to p2 (v)
    const auto vx = p2.x - p1.x;
    const auto vy = p2.y - p1.y;

    // leftward normal vector (u)
    const auto ux = -vy;
    const auto uy = vx;

    // magnitude of u
    const auto unorm = std::sqrt(ux*ux + uy*uy);

    // leftward normal vector
    const auto nx = ux / unorm;
    const auto ny = uy / unorm;

    // vector from p1 to p3
    const auto dx = p3.x - p1.x;
    const auto dy = p3.y - p1.y;

    // signed distance is the dot product (d and n)
    return dx*nx + dy*ny;
  }


  ClosePoint signMinDist2Line(const Vector2D &p1,
                              const Vector2D &p2,
                              const Vector2D &p3)
  {
    ClosePoint clpt;

    // vector from p1 to p2 (v)
    const auto vx = p2.x - p1.x;
    const auto vy = p2.y - p1.y;

    // leftward normal vector (u)
    const auto ux = -vy;
    const auto uy = vx;

    // magnitude of u
    const auto unorm = std::sqrt(ux*ux + uy*uy);

    // leftward normal vector (n)
    const auto nx = ux / unorm;
    const auto ny = uy / unorm;

    // vector from p1 to p3 (d)
    const auto dx = p3.x - p1.x;
    const auto dy = p3.y - p1.y;

    // parameteize line (t)
    const auto num = (p3.x - p1.x)*vx + (p3.y - p1.y)*vy;
    const auto denom = vx*vx + vy*vy;

    clpt.t = num / denom;

    // signed distance is the dot product (d and n)
    clpt.sign_d = dx*nx + dy*ny;

    // location of point (P) on line (not necessarily on the line segment)
    clpt.p.x = p1.x + clpt.t * vx;
    clpt.p.y = p1.y + clpt.t * vy;

    // check if  P is on segment p1 => p2
    if ((clpt.t > 0.0 and clpt.t < 1.0) or \
         rigid2d::almost_equal(clpt.t, 0.0) or \
         rigid2d::almost_equal(clpt.t, 1.0))
    {
      clpt.on_seg = true;
    }

    else
    {
      clpt.on_seg = false;
    }

    return clpt;
  }




}
