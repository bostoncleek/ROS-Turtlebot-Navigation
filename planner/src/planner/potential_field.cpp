/// \file
/// \brief Potenial fields for global planning in continous space


#include <iostream>

#include "planner/potential_field.hpp"


namespace planner
{

Vector2D descentDirection(const Vector2D &Urep, const Vector2D &Uatt)
{
  // L2 norm of gradient
  const auto ux = Urep.x + Uatt.x;
  const auto uy = Urep.y + Uatt.y;
  const auto unorm = std::sqrt(ux*ux + uy*uy);

  // descent direction
  Vector2D Ud(ux, uy);
  Ud *= 1.0 / unorm;

  return Ud;
}




PotentialField::PotentialField(obstacle_map &obs_map,
                               double eps, double step,
                               double dthresh, double qthresh,
                               double w_att, double w_rep)
                               : obs_map(obs_map),
                                 eps(eps),
                                 step(step),
                                 dthresh(dthresh),
                                 qthresh(qthresh),
                                 w_att(w_att),
                                 w_rep(w_rep),
                                 goal_reached(false)
{
}


void PotentialField::initPath(const Vector2D &start, const Vector2D &goal)
{
  std::cout << "Start: " << start << std::endl;
  std::cout << "Goal: " << goal << std::endl;
  path.push_back(start);

  q = start;
  qg = goal;
}


bool PotentialField::planPath()
{
  if(!goalReached())
  // while(!goalReached())
  {
    const Vector2D Urep = accumulateRepulsiveGradient();
    // std::cout << "Repulsive: " << Urep << std::endl;

    const Vector2D Uatt = attractiveGradient();
    // std::cout << "Attractive: " << Uatt << std::endl;

    const Vector2D Dn = descentDirection(Urep, Uatt);
    // std::cout << "Dn: " << Dn << std::endl;

    oneStepGD(Dn);
    path.push_back(q);

    // std::cout << "Current: " << q << std::endl;

    return true;
  }

  else
  {
    return false;
  }

}


Vector2D PotentialField::getCurrenPosition() const
{
  return q;
}


void PotentialField::getPath(std::vector<Vector2D> &path) const
{
  path = this->path;
}


Vector2D PotentialField::accumulateRepulsiveGradient() const
{
  Vector2D Urep(0.0, 0.0);

  // accumulate gradient for each obstacle
  for(const auto &poly : obs_map)
  {
    // closest point
    // search for closest distance
    Vector2D q0;
    auto min_dist = 1e12;

    // consider all vertices
    for(unsigned int i = 0; i < poly.size(); i++)
    {
      // vertices of current edge
      Vector2D v1, v2;

      if (i != poly.size()-1)
      {
        v1 = poly.at(i);
        v2 = poly.at(i+1);
      }

      else
      {
        v1 = poly.back();
        v2 = poly.at(0);
      }

      // min distance to from edge between vertices to current position
      ClosePoint clpt = signMinDist2Line(v1, v2, q);

      // update nominal distance and the postion of the closest point
      //  min on edge: v1 => v2
      if (clpt.on_seg and (std::fabs(clpt.sign_d) < min_dist))
      {
        q0 = clpt.p;
        min_dist = std::fabs(clpt.sign_d);
      }

      // not on egde
      // closer to v1
      else if (clpt.t < 0.0)
      {
        const auto nom_dist = euclideanDistance(v1.x, v1.y, q.x, q.y);
        if (nom_dist < min_dist)
        {
          q0 = v1;
          min_dist = nom_dist;
        }
      }

      // not on edge
      // closer to v2
      else
      {
        const auto nom_dist = euclideanDistance(v2.x, v2.y, q.x, q.y);
        if (nom_dist < min_dist)
        {
          q0 = v2;
          min_dist = nom_dist;
        }
      }

    } // end inner loop

    // repulsive gradient for obstacle
    const Vector2D Uobs = repulsiveGradient(q0, min_dist);

    // accumulate gradient
    Urep += Uobs;

  } // end outer loop

  return Urep;
}


Vector2D PotentialField::repulsiveGradient(const Vector2D &q0, double d) const
{
  // TODO check if d ~0.0

  // repulsive gradient for this obstacle
  Vector2D Urep(0.0, 0.0);

  if (d < qthresh or rigid2d::almost_equal(d, qthresh))
  {
    // std::cout << "Urep <= Qstar" << std::endl;

    // unit vector
    // current position minus closest point
    Urep.x = (q0.x - q.x) / d;
    Urep.y = (q0.y - q.y) / d;

    // weight the unit vector
    Urep *= w_rep * (1.0 / (qthresh - d)) * (1.0 / d*d);
  }

  return Urep;
}


Vector2D PotentialField::attractiveGradient() const
{
  // euclidean distance from current position (q) to goal (qd)
  const auto dg = euclideanDistance(q.x, q.y, qg.x, qg.y);

  // attractive gradient for this obstacle
  Vector2D Uatt(0.0, 0.0);

  Uatt.x = w_att * (q.x - qg.x);
  Uatt.y = w_att * (q.y - qg.y);

  if (dg > dthresh)
  {
    // std::cout << "Uatt > dstar" << std::endl;
    Uatt *= dthresh / dg;
  }

  return Uatt;
}


bool PotentialField::goalReached()
{
  // euclidean distance from current position (q) to goal (qd)
  if (euclideanDistance(q.x, q.y, qg.x, qg.y) < eps and !goal_reached)
  {
    std::cout << "Goal Reached" << std::endl;
    goal_reached = true;
    return true;
  }

  return false;
}

void PotentialField::oneStepGD(const Vector2D &Dn)
{
  q.x -= step * Dn.x;
  q.y -= step * Dn.y;
}

} // end namespace
