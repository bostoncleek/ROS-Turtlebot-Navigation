/// \file
/// \brief Global path planner usinga probabilisitc road map

#include <stdexcept>
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

  std::cout << "start ID: " << start_id << std::endl;
  std::cout << "goal ID: " << goal_id << std::endl;


  // Node n1;
  // n1.id = 1;
  // n1.f = 500.0;
  //
  // Node n2;
  // n2.id = 2;
  // n2.f = 10.0;
  //
  // Node n3;
  // n3.id = 3;
  // n3.f = 0.1;
  //
  // open_list.push_back(n1);
  // open_list.push_back(n2);
  // open_list.push_back(n3);
  //
  // std::sort(open_list.begin(), open_list.end(), SortCost());
  //
  // std::cout << "min ID: " << open_list.begin()->id << " cost: " << open_list.begin()->f << std::endl;
  //
  // auto it = std::find_if(open_list.begin(), open_list.end(), MatchesID(3));
  // std::cout << "find ID: " << it->id << std::endl;
  //
  //
  // open_list.erase(it);
  // std::cout << "min ID: " << open_list.begin()->id << " cost: " << open_list.begin()->f << std::endl;

}


bool PRMPlanner::planPath()
{

  while (!open_list.empty())
  {
    // std::cout << "---------------" <<  std::endl;

    // node with min cost
    std::sort(open_list.begin(), open_list.end(), SortCost());
    auto it = open_list.begin();
    const Node min_node = (*it);

    // remove min node from open set
    open_list.erase(it);


    curr_id = min_node.id;
    // std::cout << "min ID: " << min_node.id << std::endl;


    // goal reached
    const auto d = euclideanDistance(min_node.point.x,
                                     min_node.point.y,
                                     roadmap.at(goal_id).point.x,
                                     roadmap.at(goal_id).point.y);

    // std::cout << "D to goal: " << d << std::endl;



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

    // std::cout << "---------------" <<  std::endl;

  }
  return false;
}




void PRMPlanner::exploreNeighbors()
{
  // std::cout << "Node: " << curr_id << ", # of neighbors: " << roadmap.at(curr_id).edges.size() <<  std::endl;

  // loop across nodes adjacency list
  for(unsigned int i = 0; i < roadmap.at(curr_id).edges.size(); i++)
  {
    // index ID of a neighboring node
    const auto nid = roadmap.at(curr_id).edges.at(i).id;

    // edge(s s')
    const auto edge = roadmap.at(curr_id).edges.at(i);


    // std::cout << "Neighbor ID: " << nid <<  std::endl;



    // the neighbor is on closed list
    auto search_closed = closed_set.find(nid);
    if (search_closed != closed_set.end())
    {
      // std::cout << "Node: " << nid << " already on closed list" <<  std::endl;
      continue;
    }

    // std::cout << "Checking out " << "Node: " << nid  <<  std::endl;
    updateNode(edge);
  }
}


void PRMPlanner::updateNode(const Edge &edge)
{
  // temp node, may need to update prm node based on this one
  Node temp_node = roadmap.at(edge.id);

  // compose costs
  // g(s, s')
  temp_node.g = roadmap.at(curr_id).g + edge.d;
  // f(s') = g(s, s') + h(s')
  temp_node.f = temp_node.g;// + heuristic(edge.id);

  // set neighbor's parent
  temp_node.parent_id = curr_id;


  // if on open set compare true cost and update if needed
  auto it = std::find_if(open_list.begin(), open_list.end(), MatchesID(edge.id));
  if (it != open_list.end())
  {
    // NOTE: this means this node has already been added to the open list
    //       but may need to update its costs
    // id should be the same as node
    Node open_node = (*it);

    // std::cout << "Already on open set" <<  std::endl;

    if (temp_node.g < open_node.g)
    {
      // std::cout << "update cost" <<  std::endl;

      // remove from open set
      // open_list.erase(it);
      (*it) = temp_node;

      // std::cout << "---------------" <<  std::endl;
      // std::cout << "pre parenent" << roadmap.at(edge.id).parent_id << std::endl;

      // update prm based on temp node
      roadmap.at(edge.id) = temp_node;

      // std::cout << "pre parenent" << roadmap.at(edge.id).parent_id << std::endl;
      // std::cout << "---------------" <<  std::endl;

      // add lower cost node to open set
      // open_list.push_back(temp_node);
    }
  }

  // not on open set then add it
  else
  {
    roadmap.at(edge.id) = temp_node;
    open_list.push_back(temp_node);
  }
}



void PRMPlanner::getPath(std::vector<Vector2D> &path)
{
  // path.push_back(roadmap.at(start_id).point);
  // path.push_back(roadmap.at(goal_id).point);

  // std::cout << "Current ID: " << curr_id << std::endl;
  auto id = curr_id;

  while(id != -1)
  {
    Node node = roadmap.at(id);
    path.push_back(node.point);

    id = node.parent_id;
    // std::cout << "Parent ID: " << node.parent_id << std::endl;
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
