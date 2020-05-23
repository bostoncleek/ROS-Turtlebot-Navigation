/// \file
/// \brief Theta* using a probabilisitc road map

#include <cmath>
#include <algorithm>
#include <iostream>

#include "planner/prm_planner.hpp"


namespace planner
{

PRMPlanner::PRMPlanner(RoadMap &prm) : prm(prm)
{
  prm.getRoadMap(roadmap);
  start_id = roadmap.at(roadmap.size()-2).id;
  goal_id = roadmap.at(roadmap.size()-1).id;

  // update cost of start
  roadmap.at(start_id).f = 0.0;
  roadmap.at(start_id).g = 0.0;

  // add the start
  open_list.push_back(roadmap.at(start_id));
}


bool PRMPlanner::planPath()
{

  while (!open_list.empty())
  {
    // node with min cost
    std::sort(open_list.begin(), open_list.end(), SortCost());
    auto it = open_list.begin();
    const Node min_node = (*it);

    // remove min node from open set
    open_list.erase(it);

    curr_id = min_node.id;

    if (curr_id == goal_id)
    {
      std::cout << "Goal reached!" <<  std::endl;
      return true;
    }

    // add current node to closed set
    closed_set.insert(curr_id);

    // compose cost of neighboring cells to the current node
    // and enqueue them
    exploreNeighbors();
  }
  return false;
}

void PRMPlanner::exploreNeighbors()
{
  // loop across nodes adjacency list
  for(unsigned int i = 0; i < roadmap.at(curr_id).edges.size(); i++)
  {
    // index ID of a neighboring node
    const auto nid = roadmap.at(curr_id).edges.at(i).id;

    // edge(s s')
    const auto edge = roadmap.at(curr_id).edges.at(i);

    // the neighbor is on closed list
    auto search_closed = closed_set.find(nid);
    if (search_closed != closed_set.end())
    {
      continue;
    }
    updateNode(edge);
  }
}


void PRMPlanner::updateNode(const Edge &edge)
{
  // TODO: add edge between parent(s) and s' in strigh line

  // temp node, may need to update prm node based on this one
  Node temp_node = roadmap.at(edge.id);

  // compose costs
  // g(s, s')
  temp_node.g = roadmap.at(curr_id).g + edge.d;
  // f(s') = g(s, s') + h(s')
  temp_node.f = temp_node.g + heuristic(edge.id);

  // set neighbor's parent
  temp_node.parent_id = curr_id;

  // get parent ID and check for path optimization
  auto ps_id = roadmap.at(curr_id).parent_id;

  // there is no parent because this is the start node
  bool at_start = false;
  if (ps_id == -1)
  {
    at_start = true;
    ps_id = curr_id;
  }

  // check for line of sight from (s s')
  if (prm.stlnPathCollision(roadmap.at(ps_id).point, roadmap.at(edge.id).point))
  {
    Node parent_node = roadmap.at(ps_id);

    // distance from parent(s) to s'
    const auto d = euclideanDistance(parent_node.point.x,
                                     parent_node.point.y,
                                     roadmap.at(edge.id).point.x,
                                     roadmap.at(edge.id).point.y);

    // true cost from parent(s) to s'
    const auto gps = parent_node.g + d;

    if (gps < temp_node.g or at_start)
    {
      temp_node.g = gps;
      temp_node.f = gps + heuristic(edge.id);
      temp_node.parent_id = ps_id;


      auto it = std::find_if(open_list.begin(), open_list.end(), MatchesID(edge.id));
      if (it != open_list.end())
      {
        (*it) = temp_node;
        roadmap.at(edge.id) = temp_node;
      }

      else
      {
        roadmap.at(edge.id) = temp_node;
        open_list.push_back(temp_node);
      }
    }
  }

  else
  {
    // if on open set compare true cost and update if needed
    auto it = std::find_if(open_list.begin(), open_list.end(), MatchesID(edge.id));
    if (it != open_list.end())
    {
      // NOTE: this means this node has already been added to the open list
      //       but may need to update its costs
      // id should be the same as node
      Node open_node = (*it);

      if (temp_node.g < open_node.g)
      {
        (*it) = temp_node;
        // update prm based on temp node
        roadmap.at(edge.id) = temp_node;
      }
    }

    // not on open set then add it
    else
    {
      roadmap.at(edge.id) = temp_node;
      open_list.push_back(temp_node);
    }
  }
}


void PRMPlanner::getPath(std::vector<Vector2D> &path)
{
  auto id = curr_id;

  while(id != -1)
  {
    Node node = roadmap.at(id);
    path.push_back(node.point);

    id = node.parent_id;
  }

  std::reverse(path.begin(), path.end());
}


double PRMPlanner::heuristic(const int id)
{
  // distance from node to goal
  const auto h = euclideanDistance(roadmap.at(id).point.x,
                                   roadmap.at(id).point.y,
                                   roadmap.at(goal_id).point.x,
                                   roadmap.at(goal_id).point.y);

  return h;
}

} // end namespace
