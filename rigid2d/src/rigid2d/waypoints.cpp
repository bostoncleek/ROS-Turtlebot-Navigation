/// \file
/// \brief implementation of waypoint methods in rigid2d namespace

#include <iostream>
#include "rigid2d/waypoints.hpp"



namespace rigid2d
{

Waypoints::Waypoints(std::vector<Vector2D>  way_pts, double rot_vel, double trans_vel)
                      : drive()
{
  // store waypoints
  this->pts = way_pts;

  // headed towards 1
  idx = 1;

  // robot placed at 0
  Pose ps;
  ps.theta = 0;
  ps.x = pts.at(0).x;
  ps.y = pts.at(0).y;

  drive.reset(ps);

  // tolerances
  htol = 0.175;
  ptol = 0.1;

  // velocities
  this->rot_vel = rot_vel;
  this->trans_vel = trans_vel;
}


Twist2D Waypoints::nextWaypoint(Pose pose)
{
  Twist2D cmd;

  double h_err = waypointHeading(pose);
  h_err = normalize_angle_2PI(h_err);

  // turn
  if (std::fabs(h_err) > htol)
  {
    if (h_err <= PI and h_err >= 0)
      cmd.w = rot_vel;
    else
      cmd.w = -rot_vel;

    cmd.vx = 0;
    cmd.vy = 0;

    return cmd;
  }

  // go straight
  cmd.w = 0;
  cmd.vx = trans_vel;
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
}


double Waypoints::waypointDistance(Pose pose)
{
  double xpt = pts.at(idx).x;
  double ypt = pts.at(idx).y;

  return std::sqrt(std::pow(xpt - pose.x, 2) + std::pow(ypt - pose.y, 2));
}


double Waypoints::waypointHeading(Pose pose)
{
  double xpt = pts.at(idx).x;
  double ypt = pts.at(idx).y;
  double bearing = std::atan2(ypt - pose.y, xpt - pose.x);

  return bearing - pose.theta;
}


void Waypoints::trajectory()
{
  // current pose
  Pose pose = drive.pose();

  // current goal reached
  waypointReached(pose);

  // twist to follow
  Twist2D cmd = nextWaypoint(pose);

  // propogate kinematics
  drive.feedforward(cmd);
}






} // end namespace
















//
