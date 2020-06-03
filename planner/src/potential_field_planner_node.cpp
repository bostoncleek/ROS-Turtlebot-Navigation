/// \file
/// \brief Potenial fields for global planning in continous space
///
/// \author Boston Cleek
/// \date 5/21/20
///
/// PARAMETERS:
/// frame_id - frame of map/markers
/// frequency - frequency planner will operate at
/// eps - termination threshold for reaching goal
/// step - step size in gradient descent
/// dthresh - piecewise threshold for attractive gradient to goal
/// qthresh - obstacle range of influence
/// w_att - weighting factor the attactive component
/// w_rep - weighting factor the repulsive component
/// start_x - start x position in map coordinates
/// start_y - start y position in map coordinates
/// goal_x - goal x position in map coordinates
/// goal_y - goal y position in map coordinates
/// resolution - scale of the map coodinates (scales vertices of obstacles and start/goal)
/// bounds - boundary of map
/// obstacles - triple nested list of obstacle vertecis in map coordinates
/// PUBLISHES:
///   path (visualization_msgs::Marker): markers for start, goal, and path

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <vector>
#include <string>
#include <iostream>

#include "rigid2d/rigid2d.hpp"
#include "planner/potential_field.hpp"

static std::string frame_id;                   // frame of path marker
using rigid2d::Vector2D;

/// \brief - Fills a markers with deisred contents
/// \param path - nodes in shortest path
/// point[out] - markers for nodes in path
void pathMarkers(const std::vector<Vector2D> &path, visualization_msgs::Marker &points);



int main(int argc, char** argv)
{
  ros::init(argc, argv, "potential_field_planner");
  ros::NodeHandle nh("~");
  ros::NodeHandle node_handle;

  ros::Publisher path_pub = node_handle.advertise<visualization_msgs::Marker>("path", 1);

  // potenial field paramters
  auto eps = 0.0;                        // tolerance for reaching goal
  auto step = 0.0;                        // step size in gradient descent
  auto dthresh = 0.0;                     // distance when attractive potentail switches from conic to quadratic
  auto qthresh = 0.0;                     // repulsive potenial range of influence
  auto w_att = 0.0;                       // weight attractive potential
  auto w_rep = 0.0;                       // weight repulsive potential

  auto freq = 10.0;                       // frequency of node

  // start/goal
  auto start_x = 0.0;
  auto start_y = 0.0;
  auto goal_x = 0.0;
  auto goal_y = 0.0;

  // map parameters
  auto obs_resolution = 0.0;              // resolution of map coordinates
  XmlRpc::XmlRpcValue map_bound;          // boundaries of map
  XmlRpc::XmlRpcValue obstacles;          // triple nested vector for obstacle coordinates

  nh.getParam("frame_id", frame_id);
  nh.getParam("frequency", freq);

  // potential field parameters
  nh.getParam("eps", eps);
  nh.getParam("step", step);
  nh.getParam("dthresh", dthresh);
  nh.getParam("qthresh", qthresh);
  nh.getParam("w_att", w_att);
  nh.getParam("w_rep", w_rep);

  // start/goal
  nh.getParam("start_x", start_x);
  nh.getParam("start_y", start_y);
  nh.getParam("goal_x", goal_x);
  nh.getParam("goal_y", goal_y);

  // map parameters
  node_handle.getParam("/resolution", obs_resolution);
  node_handle.getParam("/bounds", map_bound);
  node_handle.getParam("/obstacles", obstacles);


  // const auto xmin = static_cast<double>(map_bound[0][0]) * obs_resolution;
  // const auto xmax = static_cast<double>(map_bound[0][1]) * obs_resolution;
  //
  // const auto ymin = static_cast<double>(map_bound[1][0]) * obs_resolution;
  // const auto ymax = static_cast<double>(map_bound[1][1]) * obs_resolution;

  ROS_INFO("Marker frame_id %s", frame_id.c_str());

  ROS_INFO("Successfully launched potential_field_planner node");

  // /////////////////////////////////////////////////////////////////////////////

  // store all the obstacles
  planner::obstacle_map obs_map;
  for(auto i = 0; i < obstacles.size(); i++)
  {
    planner::polygon poly;
    for(auto j = 0; j < obstacles[i].size(); j++)
    {
      Vector2D p;
      p.x = static_cast<double> (obstacles[i][j][0]) * obs_resolution;
      p.y = static_cast<double> (obstacles[i][j][1]) * obs_resolution;
      poly.push_back(p);
    }
    obs_map.push_back(poly);
  }

  // start/goal configurations
  Vector2D start(start_x * obs_resolution, start_y* obs_resolution);
  Vector2D goal(goal_x * obs_resolution, goal_y * obs_resolution);

  // ros timing
  ros::Rate rate(freq);

  // markers for start, goal, and path
  visualization_msgs::Marker path, bound;

  // // add start/goal as boundary
  bound.header.frame_id = frame_id;
  bound.header.stamp = ros::Time::now();
  bound.lifetime = ros::Duration();
  bound.ns = "boundary";
  bound.id = 0;

  bound.type = visualization_msgs::Marker::POINTS;
  bound.action = visualization_msgs::Marker::ADD;

  bound.pose.orientation.x = 0.0;
  bound.pose.orientation.y = 0.0;
  bound.pose.orientation.z = 0.0;
  bound.pose.orientation.w = 1.0;

  bound.scale.x = 0.1;
  bound.scale.y = 0.1;

  bound.color.b = 1.0f;
  bound.color.a = 1.0f;

  geometry_msgs::Point pt;
  pt.x = start.x;
  pt.y = start.y;
  bound.points.push_back(pt);

  pt.x = goal.x;
  pt.y = goal.y;
  bound.points.push_back(pt);


  // the path
  path.header.frame_id = frame_id;
  path.header.stamp = ros::Time::now();
  path.lifetime = ros::Duration();
  path.ns = "path_nodes";
  path.id = 0;

  path.type = visualization_msgs::Marker::POINTS;
  path.action = visualization_msgs::Marker::ADD;

  path.pose.orientation.x = 0.0;
  path.pose.orientation.y = 0.0;
  path.pose.orientation.z = 0.0;
  path.pose.orientation.w = 1.0;

  path.scale.x = 0.05;
  path.scale.y = 0.05;

  path.color.g = 1.0f;
  path.color.a = 1.0f;



  // potenial field global planner
  planner::PotentialField potfield(obs_map,
                                   eps, step,
                                   dthresh, qthresh,
                                   w_att, w_rep);

  // specify start/goal
  potfield.initPath(start, goal);


  while(node_handle.ok())
  {
    // find path
    // still planning
    if (potfield.planPath())
    {
      Vector2D q = potfield.getCurrenPosition();

      pt.x = q.x;
      pt.y = q.y;

      path.points.push_back(pt);

      // // retreive path
      // std::vector<Vector2D> path;
      // potfield.getPath(path);
      //
      // // display markers for path
      // pathMarkers(path, points);
    }

    path_pub.publish(path);
    path_pub.publish(bound);

    rate.sleep();
  }

  return 0;
}


void pathMarkers(const std::vector<Vector2D> &path, visualization_msgs::Marker &points)
{
  // points
  points.header.frame_id = frame_id;
  points.header.stamp = ros::Time::now();
  points.lifetime = ros::Duration();
  points.ns = "path_nodes";
  points.id = 0;

  points.type = visualization_msgs::Marker::POINTS;
  points.action = visualization_msgs::Marker::ADD;

  points.pose.orientation.x = 0.0;
  points.pose.orientation.y = 0.0;
  points.pose.orientation.z = 0.0;
  points.pose.orientation.w = 1.0;

  points.scale.x = 0.05;
  points.scale.y = 0.05;

  points.color.g = 1.0f;
  points.color.a = 1.0f;

  for(unsigned int i = 0; i < path.size(); i++)
  {
    geometry_msgs::Point pt;
    pt.x = path.at(i).x;
    pt.y =  path.at(i).y;
    pt.z = 0.0;

    points.points.push_back(pt);
  }
}

// end file
