/// \file
/// \brief Path planning on grid and constructs map as a 2D occupancy grid
///
/// \author Boston Cleek
/// \date 4/15/20
///
/// PUBLISHES:
/// map (nav_msgs::OccupancyGrid>): free/occupied/buffer zone for planning


#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include "planner/grid_map.hpp"
#include "planner/dstar_light.hpp"


using rigid2d::Vector2D;



/// \brief Places each cell in the path into a grid cell msg
/// \param path - 2D points representing the center of each cell in the path
/// cell_path[out] - array of points reprsented as grid cells
void gridCellPath(const std::vector<Vector2D> &path, nav_msgs::GridCells &cell_path);


int main(int argc, char** argv)
{
  ros::init(argc, argv, "grid_planner");
  ros::NodeHandle nh("~");
  ros::NodeHandle node_handle;

  ros::Publisher map_pub = node_handle.advertise<nav_msgs::OccupancyGrid>("map", 1);
  ros::Publisher path_pub = node_handle.advertise<nav_msgs::GridCells>("global_path", 1);
  ros::Publisher visit_pub = node_handle.advertise<nav_msgs::GridCells>("visited_cells", 1);
  ros::Publisher sg_pub = node_handle.advertise<nav_msgs::GridCells>("start_goal", 1);


  auto obs_resolution = 0.0;              // resolution of obstacle coordinates
  auto grid_resolution = 0.0;             // resolution of grid cell (cell side length)
  std::string frame_id;                   // frame of grid
  XmlRpc::XmlRpcValue map_bound;          // boundaries of map
  XmlRpc::XmlRpcValue obstacles;          // triple nested vector for obstacle coordinates
  auto bounding_radius = 0.0;             // bounding radius around robot for collisions

  nh.getParam("frame_id", frame_id);
  nh.getParam("bounding_radius", bounding_radius);
  nh.getParam("grid_resolution", grid_resolution);


  nh.getParam("resolution", obs_resolution);
  nh.getParam("bounds", map_bound);
  nh.getParam("obstacles", obstacles);


  const auto xmin = static_cast<double>(map_bound[0][0]) * obs_resolution;
  const auto xmax = static_cast<double>(map_bound[0][1]) * obs_resolution;

  const auto ymin = static_cast<double>(map_bound[1][0]) * obs_resolution;
  const auto ymax = static_cast<double>(map_bound[1][1]) * obs_resolution;


  ROS_INFO("Successfully launched grid_planner node");


  /////////////////////////////////////////////////////////////////////////////

  // load obstacles
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


  // set up the grid
  planner::GridMap gridmap(xmin, xmax,
                        ymin, ymax,
                        grid_resolution,
                        bounding_radius,
                        obs_map);

  // rviz representation of the grid
  std::vector<int8_t> map;

  // pose of grid
  geometry_msgs::Pose map_pose;
  map_pose.position.x = 0.0;
  map_pose.position.y = 0.0;
  map_pose.position.z = 0.0;

  map_pose.orientation.x = 0.0;
  map_pose.orientation.y = 0.0;
  map_pose.orientation.z = 0.0;
  map_pose.orientation.w = 1.0;


  nav_msgs::OccupancyGrid map_msg;
  map_msg.header.frame_id = frame_id;
  map_msg.info.resolution = grid_resolution;
  map_msg.info.width = planner::gridSize(xmin, xmax, grid_resolution);
  map_msg.info.height = planner::gridSize(ymin, ymax, grid_resolution);
  map_msg.info.origin = map_pose;


  // start/goal configurations
  Vector2D start(6.0 * obs_resolution, 3.0 * obs_resolution);
  Vector2D goal(20.0 * obs_resolution, 45.0 * obs_resolution);

  // visibility radius for map updates
  const auto viz_rad = 1.0*grid_resolution;
  ros::Rate rate(5);


  // convert start/goal to grid coordinates
  const planner::GridCoordinates gc_start = gridmap.world2Grid(start.x, start.y);
  const planner::GridCoordinates gc_goal = gridmap.world2Grid(goal.x, goal.y);

  const Vector2D gw_start = gridmap.grid2World(gc_start.i, gc_start.j);
  const Vector2D gw_goal = gridmap.grid2World(gc_goal.i, gc_goal.j);


  nav_msgs::GridCells sg_msg;
  sg_msg.header.frame_id = frame_id;
  sg_msg.header.stamp = ros::Time::now();
  sg_msg.cell_width = grid_resolution;
  sg_msg.cell_height = grid_resolution;

  geometry_msgs::Point ps;
  ps.x = gw_start.x;
  ps.y = gw_start.y;
  ps.z = 0.0;


  geometry_msgs::Point pg;
  pg.x = gw_goal.x;
  pg.y = gw_goal.y;
  pg.z = 0.0;

  sg_msg.cells.push_back(ps);
  sg_msg.cells.push_back(pg);


  // label cells
  gridmap.labelCells();
  // Map from grid mapper
  // gridmap.getGridViz(map);


  // number of cells visibile for map update
  const auto vizd = std::round(viz_rad / grid_resolution);

  // TODO add check if plan succeeds
  // start planning
  std::cout << "Currently Planning " <<  std::endl;
  planner::DStarLight dstar(gridmap, vizd);
  dstar.initPath(start, goal);
  dstar.planPath();



  // global path msg
  nav_msgs::GridCells path_msg;
  path_msg.header.frame_id = frame_id;
  path_msg.cell_width = 0.5 * grid_resolution;
  path_msg.cell_height = 0.5 * grid_resolution;


  // visited cells path msg
  nav_msgs::GridCells visit_msg;
  visit_msg.header.frame_id = frame_id;
  visit_msg.cell_width = 0.5 * grid_resolution;
  visit_msg.cell_height = 0.5 * grid_resolution;

  while(node_handle.ok())
  {
    ros::spinOnce();

    // plan until goal reached
    dstar.pathTraversal();

    // internal map to LPA*
    map.clear();
    dstar.getGridViz(map);

    // path viz
    std::vector<Vector2D> path;
    dstar.getPath(path);

    // path_msg.cells.clear();
    gridCellPath(path, path_msg);


    // visited cells for debugging
    std::vector<Vector2D> visted_cells;
    dstar.getVisited(visted_cells);

    // visit_msg.cells.clear();
    gridCellPath(visted_cells, visit_msg);


    ///////////////////////////////////////////////
    // publish the map
    map_msg.header.stamp = ros::Time::now();
    map_msg.info.map_load_time = ros::Time::now();
    map_msg.data = map;
    map_pub.publish(map_msg);

    // publish the visited cells
    visit_msg.header.stamp = ros::Time::now();
    visit_pub.publish(visit_msg);

    // publish the shortest path
    path_msg.header.stamp = ros::Time::now();
    path_pub.publish(path_msg);

    // publsih start/goal
    sg_pub.publish(sg_msg);
    ///////////////////////////////////////////////

    rate.sleep();
  }

  return 0;
}


void gridCellPath(const std::vector<Vector2D> &path, nav_msgs::GridCells &cell_path)
{
  cell_path.cells.resize(path.size());
  geometry_msgs::Point pt;

  for(const auto point : path)
  {
    pt.x = point.x;
    pt.y = point.y;
    pt.z = 0.0;
    cell_path.cells.push_back(pt);
  }
}











// end file
