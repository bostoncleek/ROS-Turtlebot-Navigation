/// \file
/// \brief Draw the obstacles and map boundary usig a marker array
///
/// \author Boston Cleek
/// \date 4/10/20
///
/// PUBLISHES:
///   obstacle_vertices (visualization_msgs::MarkerArray): obstacle and boundary vertices in continous Cspace
///   obstacle_edges (visualization_msgs::MarkerArray): obstacle and boundary edges in continous Cspace


#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <typeinfo>
#include <vector>
#include <string>
#include <iostream>



int main(int argc, char** argv)
{
  ros::init(argc, argv, "draw_cont_map");
  ros::NodeHandle nh("~");
  ros::NodeHandle node_handle;

  ros::Publisher vertex_pub = node_handle.advertise<visualization_msgs::MarkerArray>("obstacle_vertices", 1);
  ros::Publisher line_pub = node_handle.advertise<visualization_msgs::MarkerArray>("obstacle_edges", 1);

  std::string frame_id;
  auto resolution = 0.0;
  auto obs_total = 0;
  auto obs_total_vertices = 0;
  XmlRpc::XmlRpcValue map_bound;
  XmlRpc::XmlRpcValue obstacles;

  nh.getParam("frame_id", frame_id);
  nh.getParam("resolution", resolution);
  nh.getParam("obs_total", obs_total);
  nh.getParam("obs_total_vertices", obs_total_vertices);
  nh.getParam("bounds", map_bound);
  nh.getParam("obstacles", obstacles);


  const auto xmin = static_cast<double>(map_bound[0][0]) * resolution;
  const auto xmax = static_cast<double>(map_bound[0][1]) * resolution;

  const auto ymin = static_cast<double>(map_bound[1][0]) * resolution;
  const auto ymax = static_cast<double>(map_bound[1][1]) * resolution;

  ROS_INFO("Marker frame_id %s", frame_id.c_str());

  ROS_INFO("Successfully launched draw_cont_map node");

  /////////////////////////////////////////////////////////////////////////////

  visualization_msgs::MarkerArray obs_edges;
  visualization_msgs::MarkerArray obs_vertices;
  obs_edges.markers.resize(obs_total + 1); // account for perimeter
  obs_vertices.markers.resize(obs_total_vertices + 4); // account for corners at parimeter

  auto vertex_num = 0;

  for(auto i = 0; i < obstacles.size(); i++)
  {
    // std::cout << "Obstacle: " << i << std::endl;

    obs_edges.markers[i].header.frame_id = frame_id;
    obs_edges.markers[i].header.stamp = ros::Time::now();
    obs_edges.markers[i].lifetime = ros::Duration();
    obs_edges.markers[i].ns = "obstacle_edges";
    obs_edges.markers[i].id = i;

    obs_edges.markers[i].type = visualization_msgs::Marker::LINE_STRIP;
    obs_edges.markers[i].action = visualization_msgs::Marker::ADD;

    obs_edges.markers[i].scale.x = 0.02;

    obs_edges.markers[i].pose.orientation.x = 0.0;
    obs_edges.markers[i].pose.orientation.y = 0.0;
    obs_edges.markers[i].pose.orientation.z = 0.0;
    obs_edges.markers[i].pose.orientation.w = 1.0;


    obs_edges.markers[i].color.r = 1.0f;
    obs_edges.markers[i].color.g = 0.0f;
    obs_edges.markers[i].color.b = 0.0f;
    obs_edges.markers[i].color.a = 1.0f;



    for(auto j = 0; j < obstacles[i].size(); j++)
    {
      // cast to double
      const auto px = static_cast<double> (obstacles[i][j][0]) * resolution;
      const auto py = static_cast<double> (obstacles[i][j][1]) * resolution;


      // vertices for each obstacle
      obs_vertices.markers[vertex_num].header.frame_id = frame_id;
      obs_vertices.markers[vertex_num].header.stamp = ros::Time::now();
      obs_vertices.markers[vertex_num].lifetime = ros::Duration();
      obs_vertices.markers[vertex_num].ns = "obstacle_vertices";
      obs_vertices.markers[vertex_num].id = vertex_num;

      obs_vertices.markers[vertex_num].type = visualization_msgs::Marker::SPHERE;
      obs_vertices.markers[vertex_num].action = visualization_msgs::Marker::ADD;

      obs_vertices.markers[vertex_num].pose.position.x = px;
      obs_vertices.markers[vertex_num].pose.position.y = py;
      obs_vertices.markers[vertex_num].pose.position.z = 0.0;

      obs_vertices.markers[vertex_num].pose.orientation.x = 0.0;
      obs_vertices.markers[vertex_num].pose.orientation.y = 0.0;
      obs_vertices.markers[vertex_num].pose.orientation.z = 0.0;
      obs_vertices.markers[vertex_num].pose.orientation.w = 1.0;

      obs_vertices.markers[vertex_num].scale.x = 0.02;
      obs_vertices.markers[vertex_num].scale.y = 0.02;
      obs_vertices.markers[vertex_num].scale.z = 0.02;

      obs_vertices.markers[vertex_num].color.r = 1.0f;
      obs_vertices.markers[vertex_num].color.g = 0.0f;
      obs_vertices.markers[vertex_num].color.b = 0.0f;
      obs_vertices.markers[vertex_num].color.a = 1.0f;


      // edges for each obstacle
      geometry_msgs::Point p;
      p.x = px;
      p.y = py;
      p.z = 0.0;
      obs_edges.markers[i].points.push_back(p);

      // add the first vertex again to enclose obstacle
      if (j == obstacles[i].size() - 1)
      {
        p.x = static_cast<double> (obstacles[i][0][0]) * resolution;
        p.y = static_cast<double> (obstacles[i][0][1]) * resolution;
        obs_edges.markers[i].points.push_back(p);
      }

      vertex_num++;
      // std::cout << "[" << obstacles[i][j][0] << ", " << obstacles[i][j][1] << "]" << std::endl;
    }
  }

  // add boundaries of map
  geometry_msgs::Point p1;
  p1.x = xmin;
  p1.y = ymin;
  p1.z = 0.0;

  geometry_msgs::Point p2;
  p2.x = xmax;
  p2.y = ymin;
  p2.z = 0.0;

  geometry_msgs::Point p3;
  p3.x = xmax;
  p3.y = ymax;
  p3.z = 0.0;

  geometry_msgs::Point p4;
  p4.x = xmin;
  p4.y = ymax;
  p4.z = 0.0;

  std::vector<geometry_msgs::Point> corners = {p1, p2, p3, p4};

  for(auto i = 0; i < 4; i++)
  {
    vertex_num = obs_total_vertices + i;

    obs_vertices.markers[vertex_num].header.frame_id = frame_id;
    obs_vertices.markers[vertex_num].header.stamp = ros::Time::now();
    obs_vertices.markers[vertex_num].lifetime = ros::Duration();
    obs_vertices.markers[vertex_num].ns = "obstacle_vertices";
    obs_vertices.markers[vertex_num].id = vertex_num;

    obs_vertices.markers[vertex_num].type = visualization_msgs::Marker::SPHERE;
    obs_vertices.markers[vertex_num].action = visualization_msgs::Marker::ADD;

    obs_vertices.markers[vertex_num].pose.position.x = corners.at(i).x;
    obs_vertices.markers[vertex_num].pose.position.y = corners.at(i).y;
    obs_vertices.markers[vertex_num].pose.position.z = 0.0;

    obs_vertices.markers[vertex_num].pose.orientation.x = 0.0;
    obs_vertices.markers[vertex_num].pose.orientation.y = 0.0;
    obs_vertices.markers[vertex_num].pose.orientation.z = 0.0;
    obs_vertices.markers[vertex_num].pose.orientation.w = 1.0;

    obs_vertices.markers[vertex_num].scale.x = 0.02;
    obs_vertices.markers[vertex_num].scale.y = 0.02;
    obs_vertices.markers[vertex_num].scale.z = 0.02;

    obs_vertices.markers[vertex_num].color.r = 1.0f;
    obs_vertices.markers[vertex_num].color.g = 0.0f;
    obs_vertices.markers[vertex_num].color.b = 0.0f;
    obs_vertices.markers[vertex_num].color.a = 1.0f;
  }


  obs_edges.markers[obs_total].header.frame_id = frame_id;
  obs_edges.markers[obs_total].header.stamp = ros::Time::now();
  obs_edges.markers[obs_total].lifetime = ros::Duration();
  obs_edges.markers[obs_total].ns = "obstacle_edges";
  obs_edges.markers[obs_total].id = obs_total;

  obs_edges.markers[obs_total].type = visualization_msgs::Marker::LINE_STRIP;
  obs_edges.markers[obs_total].action = visualization_msgs::Marker::ADD;

  obs_edges.markers[obs_total].scale.x = 0.02;

  obs_edges.markers[obs_total].pose.orientation.x = 0.0;
  obs_edges.markers[obs_total].pose.orientation.y = 0.0;
  obs_edges.markers[obs_total].pose.orientation.z = 0.0;
  obs_edges.markers[obs_total].pose.orientation.w = 1.0;


  obs_edges.markers[obs_total].color.r = 1.0f;
  obs_edges.markers[obs_total].color.g = 0.0f;
  obs_edges.markers[obs_total].color.b = 0.0f;
  obs_edges.markers[obs_total].color.a = 1.0f;

  obs_edges.markers[obs_total].points.push_back(p1);
  obs_edges.markers[obs_total].points.push_back(p2);
  obs_edges.markers[obs_total].points.push_back(p3);
  obs_edges.markers[obs_total].points.push_back(p4);
  obs_edges.markers[obs_total].points.push_back(p1);


  /////////////////////////////////////////////////////////////////////////////


  while(node_handle.ok())
  {
    line_pub.publish(obs_edges);
    vertex_pub.publish(obs_vertices);
  }


  return 0;
}


// end file
