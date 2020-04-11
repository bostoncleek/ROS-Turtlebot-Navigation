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
    double d = 0.0;                 // length of edge
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
            unsigned int neighbors, unsigned int num_nodes);


    /// \brief Retreive the graph
    /// roadmap[out] - the graph
    void getRoadMap(std::vector<Node> &roadmap);

    /// \brief Print road map
    void printRoadMap();

    /// \brief Counts number of edges
    /// \returns - number of edges 
    int numEdges();

    /// \brief Construct the roadmap
    /// \param start - start configuration
    /// \param goal - goal configuration
    void constructRoadMap(const Vector2D &start, const Vector2D &goal);


  private:
    /// \brief Finds K nearest neighbors in roadmap
    /// \param query - query node
    /// neighbors[out] - index on neighbors
    void nearestNeighbors(const Node &query, std::vector<int> &neighbors);

    /// \brief Insert a node into roadmap
    /// \param q - the (x, y) location of a node
    void addNode(const Vector2D &q);

    /// \brief Add edge between two nodes
    /// \param id1 - key of first node
    /// \param id2 - key of second node
    void addEdge(int id1, int id2);



    /// \brief Generate random point in world
    Vector2D randomPoint();


    double xmin, xmax, ymin, ymax;      // bounds of the world
    double bnd_rad;                     // distance threshold between nodes/path from obstacles
    unsigned int k, n;                  // number of closest neighbors, number of nodes
    std::vector<Node> nodes;            // graph representation of road map

  };





} // end namespace




#endif
