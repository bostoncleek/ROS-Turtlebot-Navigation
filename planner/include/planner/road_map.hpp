#ifndef ROAD_MAP_HPP
#define ROAD_MAP_HPP
/// \file
/// \brief Probabilisitc road maps


#include <vector>
#include <iosfwd>
#include <unordered_set>

#include <rigid2d/utilities.hpp>
#include <rigid2d/rigid2d.hpp>



namespace planner
{
  using rigid2d::sampleUniformDistribution;
  using rigid2d::Vector2D;
  using rigid2d::euclideanDistance;


  typedef std::vector<Vector2D> polygon;    // convex polygon, vertices counter-clockwise
  typedef std::vector<polygon> obs_map;     // constain all polygons in Cspace


  /// \brief Egde connecting two nodes
  struct Edge
  {
    int id = -1;                    // id of node edge connects to
    double distance = 0.0;          // length of edge
  };


  /// \brief Node in the road map
  struct Node
  {
    int id = -1;                            // nodes id, index in nodes
    Vector2D point;                         // (x,y) location in world
    std::vector<Edge> edges;                // edges to other nodes
    std::unordered_set<int> id_set;         // id of adjacent nodes

    /// \brief Checks if edge exists in the id_map
    /// \param id - id of node connected to
    /// \return true if node is connected to id
    bool edgeExists(int id)
    {
      const auto search = id_set.find(id);
      return (search == id_set.end()) ? false : true;
    }
  };



  class RoadMap
  {
  public:


  private:

  };





} // end namespace




#endif
