/// \file
/// \brief Library for generating control (body twist) between waypoints

#include <iostream>
#include <algorithm>
#include "rigid2d/waypoints.hpp"



namespace rigid2d
{

Waypoints::Waypoints(const std::vector<Vector2D> &way_pts, double rot_vel, double trans_vel)
        : pts(way_pts),
          idx(0),
          ctr(0),
          htol(0.02),
          ptol(0.025),
          rot_vel(rot_vel),
          trans_vel(trans_vel),
          k_rot(0.0),
          k_trans(0.0),
          cycle_complete(false)
{
}


void Waypoints::setGain(double k_rot)
{
  this->k_rot = k_rot;
}



Twist2D Waypoints::nextWaypoint(Pose pose)
{
  Twist2D cmd;

  // current goal reached
  waypointReached(pose);


  double h_err = waypointHeading(pose);
  h_err = normalize_angle_PI(h_err);


  // go straight
  if (std::fabs(h_err) < htol)
  {
    cmd.w = 0;
    cmd.vx = trans_vel;
    cmd.vy = 0;

    return cmd;
  }

  // turn
  if (h_err > 0)
    cmd.w = rot_vel;
  else
    cmd.w = -rot_vel;

  cmd.vx = 0;
  cmd.vy = 0;

  return cmd;
}


Twist2D Waypoints::nextWaypointClosedLoop(Pose pose)
{
  Twist2D cmd;

  // current goal reached
  waypointReached(pose);

  // stop after one cycle
  if (cycle_complete)
  {
    return cmd;
  }


  const auto h_err = normalize_angle_PI(waypointHeading(pose));
  // const auto p_err = waypointDistance(pose);


  // go straight
  // cant go in reverse
  if (std::fabs(h_err) < htol)
  {
    cmd.vx = trans_vel;
    cmd.w = 0;
    cmd.vy = 0;

    return cmd;
  }


  // turn
  cmd.w = k_rot * h_err;
  cmd.w = std::clamp(cmd.w, -rot_vel, rot_vel);

  cmd.vx = 0;
  cmd.vy = 0;

  return cmd;
}



void Waypoints::waypointReached(Pose pose)
{
  if (waypointDistance(pose) < ptol)
  {
    std::cout << "Reached waypoint: " << idx << std::endl;
    incrementWaypoint();
  }
}


void Waypoints::incrementWaypoint()
{
  idx++;
  ctr++;

  // last waypoint reached head back to start
  if (idx % pts.size() == 0)
  {
    idx = 0;
  }

  if (ctr == static_cast<int> (pts.size()+1))
  {
    cycle_complete = true;
    std::cout << "Once Cyle Complete" << std::endl;
    return;
  }

  std::cout << "Headed to waypoint: " << idx << " at ["
  << pts.at(idx).x << " " << pts.at(idx).y << "]" << std::endl;
}


double Waypoints::waypointDistance(Pose pose) const
{
  double xpt = pts.at(idx).x;
  double ypt = pts.at(idx).y;

  return std::sqrt(std::pow(xpt - pose.x, 2) + std::pow(ypt - pose.y, 2));
}


double Waypoints::waypointHeading(Pose pose) const
{
  double xpt = pts.at(idx).x;
  double ypt = pts.at(idx).y;
  double bearing = std::atan2(ypt - pose.y, xpt - pose.x);

  return bearing - pose.theta;
}






} // end namespace
















//
