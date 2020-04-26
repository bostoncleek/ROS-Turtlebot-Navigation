/// \file
/// \brief Global path planner usinga probabilisitc road map

#include <stdexcept>
#include <cmath>
#include <iostream>


#include "planner/prm_planner.hpp"


namespace planner
{

PRMPlanner::PRMPlanner(RoadMap &prm) : prm(prm)
{
  prm.getRoadMap(roadmap);
  start_id = roadmap.size()-2;
  goal_id = roadmap.size()-1;
  curr_id = start_id;

  // update cost of start
  roadmap.at(start_id).f = 0.0;
  roadmap.at(start_id).g = 0.0;

  // enqueue the start
  open_set.insert(start_id);

  // enqueue the start
  enqueueOpenSet();



  Node n1;
  n1.id = 1;
  n1.f = 0.5;

  Node n2;
  n2.id = 2;
  n2.f = 10.0;

  open.insert(n1);
  open.insert(n2);


  std::cout << "min ID: " << open.begin()->id << " cost: " << open.begin()->f << std::endl;
  std::cout << "find ID: " << open.find(2)->f << std::endl;



  auto it = open.find(2);
  std::cout << "found ID: "  << it->id << std::endl;
  open.erase(it);

  std::cout << "min ID: " << open.begin()->id << " cost: " << open.begin()->f << std::endl;

  it = open.find(1);
  std::cout << "found ID: "  << it->id << std::endl;
  open.erase(it);
}


bool PRMPlanner::planPath()
{

  while (!q.empty())
  {
    // node with min cost
    const Node min_node = q.top();
    q.pop();
    curr_id = min_node.id;
    open_set.erase(curr_id);
    std::cout << "min ID: " << min_node.id << std::endl;

    // goal reached
    const auto d = euclideanDistance(min_node.point.x,
                                     min_node.point.y,
                                     roadmap.at(goal_id).point.x,
                                     roadmap.at(goal_id).point.y);

    if (rigid2d::almost_equal(d, 0.0))
    {
      std::cout << "Goal reached!" <<  std::endl;
      return true;
    }

    // add current node to closed set
    closed_set.insert(curr_id);

    // compose cost of neighboring cells to the current node
    // and enqueue them
    enqueueNeighbors();

  }
  return false;
}


void PRMPlanner::enqueueNeighbors()
{
  // loop across nodes adjacency list
  for(unsigned int i = 0; i < roadmap.at(curr_id).edges.size(); i++)
  {
    // index ID of a neighboring node
    const auto neighbor_id = roadmap.at(curr_id).edges.at(i).id;

    // compose the cost of the neighbor
    Node temp_node = composeCost(neighbor_id);

    if (examineNeighbor(temp_node))
    {
      // update the node in the prm with the temp node
      roadmap.at(neighbor_id) = temp_node;

      // add temp_node to open set
      open_set.insert(neighbor_id);
    }
  }

  // heapify the open list
  enqueueOpenSet();
}


Node PRMPlanner::composeCost(const int id)
{
  // create a temporary node
  Node node = roadmap.at(id);

  // update node's parent, which is the current min node
  node.parent_id = curr_id;

  // the cost to move from the current cell to the neighbor is the
  // distance between the two nodes
  const auto dcn = euclideanDistance(roadmap.at(curr_id).point.x,
                                     roadmap.at(curr_id).point.y,
                                     roadmap.at(id).point.x,
                                     roadmap.at(id).point.y);

  // true cost
  const auto true_cost = roadmap.at(curr_id).g + dcn;


  // update cost of neighbor node
  node.g = true_cost;
  node.f = true_cost + heuristic(id);

  return node;
}



bool PRMPlanner::examineNeighbor(const Node &node)
{
  // on closed list
  auto search_closed = closed_set.find(node.id);
  if (search_closed != closed_set.end())
  {
    return false;
  }

  // on open list
  auto search_open = open_set.find(node.id);
  if (search_open != open_set.end())
  {
    // NOTE: this means this node has already been added to the open list
    //       but may need to update its costs
    // id should be the same as node
    const auto id = (*search_open);

    // compare true cost
    // node on the open list is already better than or equal to the temp
    if (roadmap.at(id).g < node.g or rigid2d::almost_equal(node.g, roadmap.at(id).g))
    {
      return false;
    }


    // temp is better than node on open list so update it
    else if (roadmap.at(id).g > node.g)
    {
      roadmap.at(id).g = node.g;
      return false;
    }
  }

  // not on open or closed
  // it will be added to the open list
  return true;
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



void PRMPlanner::enqueueOpenSet()
{
  // TODO: find more effecient method
  //        this is required incase a node on the queue is updated

  while(!q.empty())
  {
    q.pop();
  }


  for(const auto &id : open_set)
  {
    // std::cout << "id: "<< id << std::endl;
    q.push(roadmap.at(id));
  }
}









} // end namespace
