#ifndef PRM_PLANNER_HPP
#define PRM_PLANNER_HPP
/// \file
/// \brief Global path planner usinga probabilisitc road map

#include <vector>
#include <iosfwd>
#include <queue>
#include <unordered_set>
#include <set>

#include "planner/planner_utilities.hpp"
#include "planner/road_map.hpp"


namespace planner
{
  using rigid2d::euclideanDistance;


  struct CompareCost
  {
    bool operator()(const Node &a, const Node &b)
    {
      return a.f > b.f;
    }
  };


  struct CompareNodes
  {
    using is_transparent = void;

    bool operator()(const Node &a, const Node &b) const
    {
      return a.f < b.f;
    }

    bool operator()(const int id, const Node &node) const
    {
      return id > node.id;
    }

    bool operator()(const Node &node, const int id) const
    {
      return node.id < id;
    }
  };



  class PRMPlanner
  {
  public:
    /// \brief Construct a global path planner
    /// \param prm - a probabilisitc road map
    PRMPlanner(RoadMap &prm);

    /// \brief Plans a path from start to goal on the PRM
    /// \return true if path is founc
    bool planPath();

    /// \brief Add/examines the neighboring cells of the current min node
    void enqueueNeighbors();

    /// \brief Compose the cost of a neighbor node
    /// \param id - the node ID
    /// \return - a temporary node, need to examine before added to open list
    Node composeCost(const int id);

    /// \brief Determines whether a node should be added to the open list
    /// \param node - the neighbor node
    /// \return true if the node should be added to the open list
    bool examineNeighbor(const Node &node);

    /// \brief Compose the heuristic cost of a node
    /// \param id - the node ID
    /// \return - heuristic cost of node
    double heuristic(const int id);

    /// \brief Removes all nodes and enqueues all nodes on the open set
    void enqueueOpenSet();



  private:
    RoadMap prm;                              // probabilisitc road map
    std::vector<Node> roadmap;                // graph structure of road map
    std::unordered_set<int> open_set;         // open set
    std::unordered_set<int> closed_set;       // closed set
    Node curr_node;
    int start_id, goal_id, curr_id;           // ID of start/goal/current node in roadmap

    std::set<Node, CompareNodes> open;


    // queue for finding the min node
    // constains the nodes on the open list
    std::priority_queue<Node, std::vector<Node>, CompareCost> q;


  };
} // end namespace


#endif
