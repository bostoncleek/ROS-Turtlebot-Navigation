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


  struct SortCost
  {
    bool operator()(const Node &a, const Node &b) const
    {
      return a.f < b.f;
    }
  };


  struct MatchesID
  {
    int id;
    MatchesID(int id): id(id) {}

    bool operator()(const Node &n) const
    {
        return n.id == id;
    }
  };


  /// \brief Theta* shortest path planner
  class PRMPlanner
  {
  public:
    /// \brief Construct a global path planner
    /// \param prm - a probabilisitc road map
    PRMPlanner(RoadMap &prm);

    /// \brief Plans a path from start to goal on the PRM
    /// \return true if path is founc
    bool planPath();

    /// \brief Coordinates of nodes in path
    /// path[out] - (x/y) locations of each node
    void getPath(std::vector<Vector2D> &path);

  private:
    /// \brief Add/examines the neighboring cells of the current min node
    void exploreNeighbors();

    /// \brief
    /// \param edge - edge between current node and a neighbor (s, s')
    void updateNode(const Edge &edge);

    /// \brief Compose the heuristic cost of a node
    /// \param id - the node ID
    /// \return - heuristic cost of node
    double heuristic(const int id);


    RoadMap prm;                              // probabilisitc road map
    std::vector<Node> roadmap;                // graph structure of road map
    std::vector<Node> open_list;              // open list, nodes currently being considered
    std::unordered_set<int> closed_set;       // closed set, nodes that were once on the open list
    int start_id, goal_id, curr_id;           // ID of start/goal/current node in roadmap





  };
} // end namespace


#endif
