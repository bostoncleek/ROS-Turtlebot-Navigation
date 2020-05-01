/// \file
/// \brief Probabilisitc road map path planning and
///        draws the path using marker array
///
/// \author Boston Cleek
/// \date 4/11/20
///
/// PUBLISHES:
/// prm (visualization_msgs::MarkerArray): obstacle and boundary vertices in continous Cspace
/// shortest_path (visualization_msgs::Marker): points and edges for shortest path 



#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <typeinfo>
#include <vector>
#include <string>
#include <iostream>

#include "planner/road_map.hpp"
#include "planner/prm_planner.hpp"

using rigid2d::Vector2D;

static std::string frame_id;                   // frame of path marker


/// \brief - Fills a marker array with deisred contents
/// \param nodes - nodes and edges in prm
/// \param color - color of spheres and lines
/// \param prm - set to true if marker array is for prm
/// nodes_array[out] - sphere markers for nodes
/// edges_array[out] - line list markers for edges
void prmMarkerArray(const std::vector<planner::Node> &nodes,
                     visualization_msgs::MarkerArray &nodes_array,
                     visualization_msgs::MarkerArray &edges_array,
                     std::string color);


/// \brief - Fills a markers with deisred contents
/// \param path - nodes in shortest path
/// point[out] - markers for nodes
/// line_strip[out] - line strip markers for edges between nodes
void pathMarkers(const std::vector<Vector2D> &path,
                 visualization_msgs::Marker &points,
                 visualization_msgs::Marker &line_strip);


int main(int argc, char** argv)
{
  ros::init(argc, argv, "prm_global_planner");
  ros::NodeHandle nh("~");
  ros::NodeHandle node_handle;

  ros::Publisher prm_pub = node_handle.advertise<visualization_msgs::MarkerArray>("prm", 1);
  ros::Publisher path_pub = node_handle.advertise<visualization_msgs::Marker>("shortest_path", 1);


  auto resolution = 0.0;                  // resolution of map coordinates
  XmlRpc::XmlRpcValue map_bound;          // boundaries of map
  XmlRpc::XmlRpcValue obstacles;          // triple nested vector for obstacle coordinates

  auto bounding_radius = 0.0;              // bounding radius around robot for collisions
  auto nearest_neighbors = 0;             // number of NN in PRM
  auto num_nodes = 0;                     // number of nodes in PRM


  nh.getParam("frame_id", frame_id);
  nh.getParam("bounding_radius", bounding_radius);
  nh.getParam("nearest_neighbors", nearest_neighbors);
  nh.getParam("num_nodes", num_nodes);

  nh.getParam("resolution", resolution);
  nh.getParam("bounds", map_bound);
  nh.getParam("obstacles", obstacles);


  const auto xmin = static_cast<double>(map_bound[0][0]) * resolution;
  const auto xmax = static_cast<double>(map_bound[0][1]) * resolution;

  const auto ymin = static_cast<double>(map_bound[1][0]) * resolution;
  const auto ymax = static_cast<double>(map_bound[1][1]) * resolution;

  ROS_INFO("Marker frame_id %s", frame_id.c_str());

  ROS_INFO("Successfully launched prm_global_planner node");

  /////////////////////////////////////////////////////////////////////////////

  // store all the obstacles
  planner::obstacle_map obs_map;
  for(auto i = 0; i < obstacles.size(); i++)
  {
    planner::polygon poly;
    for(auto j = 0; j < obstacles[i].size(); j++)
    {
      Vector2D p;
      p.x = static_cast<double> (obstacles[i][j][0]) * resolution;
      p.y = static_cast<double> (obstacles[i][j][1]) * resolution;
      poly.push_back(p);
    }
    obs_map.push_back(poly);
  }


  planner::RoadMap prm(xmin, xmax,
                       ymin, ymax,
                       bounding_radius,
                       nearest_neighbors,
                       num_nodes,
                       obs_map);


  // declare the start/goal
  Vector2D start(2.1, 0.9);
  Vector2D goal(6.0, 13.5);
  // Vector2D goal(2.1, 7.8);


  // construct the prm
  prm.constructRoadMap(start, goal);
  std::vector<planner::Node> nodes;
  prm.getRoadMap(nodes);

  // prm.printRoadMap();

  // marker arrays for prm
  visualization_msgs::MarkerArray nodes_array;
  visualization_msgs::MarkerArray edges_array;
  nodes_array.markers.resize(nodes.size());
  edges_array.markers.resize(nodes.size());

  prmMarkerArray(nodes, nodes_array, edges_array, "green");


  // find shortest path
  planner::PRMPlanner plan(prm);
  plan.planPath();

  // collection of node comprising the path
  std::vector<Vector2D> path;
  plan.getPath(path);

  // marker for shortest path
  visualization_msgs::Marker points, line_strip;
  pathMarkers(path, points, line_strip);


  // for(const auto &elem: path)
  // {
  //   std::cout << elem << std::endl;
  // }


  while(node_handle.ok())
  {

   prm_pub.publish(nodes_array);
   prm_pub.publish(edges_array);

   path_pub.publish(points);
   path_pub.publish(line_strip);
  }
  return 0;
}





void prmMarkerArray(const std::vector<planner::Node> &nodes,
                     visualization_msgs::MarkerArray &nodes_array,
                     visualization_msgs::MarkerArray &edges_array,
                     std::string color)
{

  for(unsigned int i = 0; i < nodes.size(); i++)
  {

    // coordinates of node
    const auto ndx = nodes.at(i).point.x;
    const auto ndy = nodes.at(i).point.y;
    geometry_msgs::Point nd_pt;
    nd_pt.x = ndx;
    nd_pt.y = ndy;
    nd_pt.z = 0.0;


    // add spheres for nodes
    nodes_array.markers[i].header.frame_id = frame_id;
    nodes_array.markers[i].header.stamp = ros::Time::now();
    nodes_array.markers[i].lifetime = ros::Duration();
    nodes_array.markers[i].ns = "nodes";
    nodes_array.markers[i].id = i;

    nodes_array.markers[i].type = visualization_msgs::Marker::SPHERE;
    nodes_array.markers[i].action = visualization_msgs::Marker::ADD;

    nodes_array.markers[i].pose.position.x = ndx;
    nodes_array.markers[i].pose.position.y = ndy;
    nodes_array.markers[i].pose.position.z = 0.0;

    nodes_array.markers[i].pose.orientation.x = 0.0;
    nodes_array.markers[i].pose.orientation.y = 0.0;
    nodes_array.markers[i].pose.orientation.z = 0.0;
    nodes_array.markers[i].pose.orientation.w = 1.0;

    nodes_array.markers[i].scale.x = 0.1;
    nodes_array.markers[i].scale.y = 0.1;
    nodes_array.markers[i].scale.z = 0.1;


    if (color == "green")
    {
      nodes_array.markers[i].color.r = 0.0f;
      nodes_array.markers[i].color.g = 1.0f;
      nodes_array.markers[i].color.b = 0.0f;
      nodes_array.markers[i].color.a = 1.0f;
    }

    else if (color == "blue")
    {
      nodes_array.markers[i].color.r = 0.0f;
      nodes_array.markers[i].color.g = 0.0f;
      nodes_array.markers[i].color.b = 1.0f;
      nodes_array.markers[i].color.a = 1.0f;
    }


    // add line strip for edges
    edges_array.markers[i].header.frame_id = frame_id;
    edges_array.markers[i].header.stamp = ros::Time::now();
    edges_array.markers[i].lifetime = ros::Duration();
    edges_array.markers[i].ns = "edges";
    edges_array.markers[i].id = i;

    edges_array.markers[i].type = visualization_msgs::Marker::LINE_LIST;
    edges_array.markers[i].action = visualization_msgs::Marker::ADD;

    edges_array.markers[i].scale.x = 0.02;

    edges_array.markers[i].pose.orientation.x = 0.0;
    edges_array.markers[i].pose.orientation.y = 0.0;
    edges_array.markers[i].pose.orientation.z = 0.0;
    edges_array.markers[i].pose.orientation.w = 1.0;

    if (color == "green")
    {
      edges_array.markers[i].color.r = 0.0f;
      edges_array.markers[i].color.g = 1.0f;
      edges_array.markers[i].color.b = 0.0f;
      edges_array.markers[i].color.a = 1.0f;
    }

    else if (color == "blue")
    {
      edges_array.markers[i].color.r = 0.0f;
      edges_array.markers[i].color.g = 0.0f;
      edges_array.markers[i].color.b = 1.0f;
      edges_array.markers[i].color.a = 1.0f;
    }

    // loop across adjacency list
    for(unsigned int j = 0; j < nodes.at(i).edges.size(); j++)
    {
      const auto adj_id = nodes.at(i).edges.at(j).id;

      geometry_msgs::Point adj_pt; // adjacent node point
      adj_pt.x = nodes.at(adj_id).point.x;
      adj_pt.y = nodes.at(adj_id).point.y;
      adj_pt.z = 0.0;

      edges_array.markers[i].points.push_back(nd_pt);
      edges_array.markers[i].points.push_back(adj_pt);

    } // end inner loop
  } // end outer loop
}


void pathMarkers(const std::vector<Vector2D> &path,
                 visualization_msgs::Marker &points,
                 visualization_msgs::Marker &line_strip)
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

  points.scale.x = 0.2;
  points.scale.y = 0.2;

  points.color.b = 1.0f;
  points.color.a = 1.0f;

  // line strip
  line_strip.header.frame_id = frame_id;
  line_strip.header.stamp = ros::Time::now();
  line_strip.lifetime = ros::Duration();
  line_strip.ns = "path_nodes";
  line_strip.id = 1;

  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.action = visualization_msgs::Marker::ADD;

  line_strip.pose.orientation.x = 0.0;
  line_strip.pose.orientation.y = 0.0;
  line_strip.pose.orientation.z = 0.0;
  line_strip.pose.orientation.w = 1.0;

  line_strip.scale.x = 0.1;

  line_strip.color.b = 1.0f;
  line_strip.color.a = 1.0f;


  for(unsigned int i = 0; i < path.size(); i++)
  {
    geometry_msgs::Point pt;
    pt.x = path.at(i).x;
    pt.y =  path.at(i).y;
    pt.z = 0.0;

    points.points.push_back(pt);
    line_strip.points.push_back(pt);
  }
}























// end file
