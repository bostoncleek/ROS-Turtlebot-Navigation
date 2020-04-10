#ifndef ROAD_MAP_HPP
#define ROAD_MAP_HPP
/// \file
/// \brief Probabilisitc road maps


#include <vector>
#include <iosfwd>
#include <unordered_set>

#include <rigid2d/utilities.hpp>
#include <rigid2d/rigid2d.hpp>



namespace navigation
{

  using rigid2d::sampleUniformDistribution;
  using rigid2d::Vector2D;
  using rigid2d::euclideanDistance;


  /// \brief Egde connecting two nodes
  struct Edge
  {
    int next_id = -1;                    // id of node edge connects to
    double d = 0.0;                     // length of edge
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


  /// \brief Map of circles
  struct FeatureMap
  {
    std::vector<double> cx;         // circles x position
    std::vector<double> cy;         // circles y position
    std::vector<double> r;          // circles radius
  };


  /// \brief Compose a road map from start to goal
  class RoadMap
  {
  public:
    /// \brief Compose road map
    /// \param xmin - lower x world bound
    /// \param xmax - upper x world bound
    /// \param ymin - lower y world bound
    /// \param ymax - upper y world bound
    /// \param obs_dist - distance threshold between path and obstacles
    /// \param node_dist - distance threshold between nodes on road map
    /// \param neighbors - number of closest neighbors examine for each configuration
    /// \param num_nodes - number of nodes to put in road map
    RoadMap(double xmin, double xmax,
            double ymin, double ymax,
            double obs_dist, double node_dist,
            unsigned int neighbors, unsigned int num_nodes);

    /// \brief Construct the road map
    /// \param map - current map
    /// \param start - start configuration
    /// \param goal - goal configuration
    void constructRoadMap(const FeatureMap &map, const Vector2D &start, const Vector2D &goal);

    void printRoadMap();



  private:
    /// \brief Finds K nearest neighbors in road map
    /// \param query - query node
    /// neighbors[out] - index on neighbors
    void nearestNeighbors(const Node &query, std::vector<int> &neighbors);

    /// \brief Check whether a point is in free space
    /// \param map - current map
    /// \param q - random configuration
    /// \return - true if free space
    bool isFreeSpace(const FeatureMap &map, const Vector2D &q);

    /// \brief Checks for collison between obstacles and edges in road map
    /// \param map - current map
    /// \param p1 - first end point of line segment
    /// \param p2 - second end point of line segment
    /// \return - true if collison
    bool edgeCollision(const FeatureMap &map, const Vector2D &p1, const Vector2D &p2);

    /// \brief Generate random point in world
    Vector2D randomPoint();

    double xmin, xmax, ymin, ymax;      // bounds of the world
    double obs_dist;                    // distance away from obstacles
    double node_dist;                   // min distance between nodes
    unsigned int k, n;                  // number of closest neighbors, number of nodes
    std::vector<Node> nodes;            // graph representation of road map

  };




} // end namespace




#endif
