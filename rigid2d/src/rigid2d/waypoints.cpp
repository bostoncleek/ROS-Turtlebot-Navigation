/// \file
/// \brief Library for generating control (body twist) between waypoints

#include <iostream>
#include "rigid2d/waypoints.hpp"



namespace rigid2d
{

Waypoints::Waypoints(const std::vector<Vector2D> &way_pts, double rot_vel, double trans_vel)
{
  // store waypoints
  this->pts = way_pts;

  // assume NOT starting at 0
  // head to 0
  idx = 0;

  // tolerances
  htol = 0.005;   // heading tolerance
  ptol = 0.1;     // distance to goal tolerance

  // velocities
  this->rot_vel = rot_vel;
  this->trans_vel = trans_vel;
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

  // last waypoint reached head back to start
  if (idx % pts.size() == 0)
    idx = 0;

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
