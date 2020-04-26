#ifndef ROAD_MAP_HPP
#define ROAD_MAP_HPP
/// \file
/// \brief Probabilisitc road maps


#include <vector>
#include <iosfwd>
#include <unordered_set>

#include <rigid2d/utilities.hpp>
#include "planner/planner_utilities.hpp"

namespace planner
{

  using rigid2d::sampleUniformDistribution;
  using rigid2d::Vector2D;
  using rigid2d::euclideanDistance;


  /// \brief Egde connecting two nodes
  struct Edge
  {
    int id = -1;                    // id of node edge connects to
    double d = 0.0;                 // length of edge
  };


  /// \brief Node in the road map
  struct Node
  {
    // used for road map construction
    int id = -1;                            // nodes id, index in nodes
    Vector2D point;                         // (x,y) location in world
    std::vector<Edge> edges;                // edges to other nodes
    std::unordered_set<int> id_set;         // id of adjacent nodes

    // used for planning
    int parent_id = -1;                     // ID of parents node
    double f = 0.0;                         // total cost = true cost + heuristic cost
    double g = 0.0;                         // true cost from start to node


    /// \brief Checks if edge exists in the id_map
    /// \param id - id of node connected to
    /// \return true if node is connected to id
    bool edgeExists(int id)
    {
      const auto search = id_set.find(id);
      return (search == id_set.end()) ? false : true;
    }
  };


  /// \brief Check whether line segment intersects a polygon
  /// \param poly - plygon to examine
  /// \param p1 - first bound of line segment
  /// \param p1 - second bound of line segment
  /// \return - true if intersects polygon
  bool lnSegIntersectPolygon(const polygon &poly,
                             const Vector2D &p1,
                             const Vector2D &p2);

   /// \brief Compose graph for global planning
  class RoadMap
  {
  public:
    /// \brief Compose road map
    /// \param xmin - lower x world bound
    /// \param xmax - upper x world bound
    /// \param ymin - lower y world bound
    /// \param ymax - upper y world bound
    /// \param bnd_rad - bounding radius around robot
    /// \param neighbors - number of closest neighbors examine for each configuration
    /// \param num_nodes - number of nodes to put in road map
    RoadMap(double xmin, double xmax,
            double ymin, double ymax,
            double bnd_rad,
            unsigned int neighbors, unsigned int num_nodes,
            obstacle_map obs_map);

    /// \brief Retreive the graph
    /// roadmap[out] - the graph
    void getRoadMap(std::vector<Node> &roadmap) const;

    /// \brief Print road map
    void printRoadMap() const;

    /// \brief Construct the roadmap
    /// \param start - start configuration
    /// \param goal - goal configuration
    void constructRoadMap(const Vector2D &start, const Vector2D &goal);

    /// \brief Check whether an edge between two nodes is feasible
    /// \param p1 - first bound of edge
    /// \param p1 - second bound of edge
    /// \return - true no collision between edge on polygons
    bool stlnPathCollision(const Vector2D &p1, const Vector2D &p2) const;

  private:
    /// \brief Adds the start and goal nodes
    /// \param start - start configuration
    /// \param goal - goal configuration
    /// \return - true if start/goal nodes are connected to prm
    bool addStartGoalConfig(const Vector2D &start, const Vector2D &goal);

    /// \brief Finds K nearest neighbors in roadmap
    /// \param query - query node
    /// neighbors[out] - index on neighbors
    void nearestNeighbors(const Node &query, std::vector<int> &neighbors) const;

    /// \brief Check if node collides with boundaries of map
    /// \param q - the (x, y) location of a node
    /// \return - true if q collides with boundary
    bool collideWalls(const Vector2D &q) const;

    /// \brief Check whether a point is in free space
    /// \param q - random configuration
    /// \return - true if free space
    bool isFreeSpace(const Vector2D &q) const;

    /// \brief Check whether a point isnside or close to a polygon
    /// \param poly - plygon to examine
    /// \param q - random configuration
    /// \return - true if inside or close to polygon
    bool ptInsidePolygon(const polygon &poly, const Vector2D &q) const;

    /// \brief Check whether line segment comes within
    ///        a distance theshold of a polygon
    /// \param poly - plygon to examine
    /// \param p1 - first bound of line segment
    /// \param p1 - second bound of line segment
    /// \return - true if line segment is within tolerance of a polygon
    bool lnSegClose2Polygon(const polygon &poly,
                            const Vector2D &p1,
                            const Vector2D &p2) const;

    /// \brief Insert a node into roadmap
    /// \param q - the (x, y) location of a node
    /// \return - node ID
    int addNode(const Vector2D &q);

    /// \brief Add edge between two nodes
    /// \param id1 - key of first node
    /// \param id2 - key of second node
    void addEdge(int id1, int id2);

    /// \brief Generate random point in world
    Vector2D randomPoint() const;


    double xmin, xmax, ymin, ymax;      // bounds of the world
    double bnd_rad;                     // distance threshold between nodes/path from obstacles
    unsigned int k, n;                  // number of closest neighbors, number of nodes
    obstacle_map obs_map;               // collection of all the polygons

    // graph representation of road map
    // start node is at position n-2
    // goal node is at position n-1
    std::vector<Node> nodes;
  };

} // end namespace


#endif
