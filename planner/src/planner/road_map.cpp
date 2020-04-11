/// \file
/// \brief Probabilisitc road maps

#include <stdexcept>
#include <cmath>
#include <iostream>

#include "planner/road_map.hpp"


namespace planner
{

RoadMap::RoadMap(double xmin, double xmax,
                 double ymin, double ymax,
                 double bnd_rad,
                 unsigned int neighbors, unsigned int num_nodes)
                    : xmin(xmin),
                      xmax(xmax),
                      ymin(ymin),
                      ymax(ymax),
                      bnd_rad(bnd_rad),
                      k(neighbors),
                      n(num_nodes)
{
  if (n <= k)
  {
    throw std::invalid_argument("Number of nodes in road map less than nearest neighbors");
  }
}


void RoadMap::getRoadMap(std::vector<Node> &roadmap)
{
  roadmap = nodes;
}


void RoadMap::printRoadMap()
{
  for(const auto &nd : nodes)
  {
    // std::cout << nd.id << ": " << nd.point << "| ";
    std::cout << nd.id << "| ";


    for(const auto &ed : nd.edges)
    {
      std::cout << "id: " <<  ed.id << " ";
    }
    std::cout << std::endl;
  }
}


int RoadMap::numEdges()
{
  auto num = 0;
  for(const auto &nd : nodes)
  {
    num += nd.edges.size();
  }
  return num;
}


void RoadMap::constructRoadMap(const Vector2D &start, const Vector2D &goal)
{

  // TODO: check if nodes are away from perimater walls


  // clear prvious graph
  nodes.clear();

  // add nodes
  while(nodes.size() < n)
  {
    Vector2D q = randomPoint();

    // if(isFreeSpace(map, q))
    {
      addNode(q);
    }
  } // end while loop


  // add edges
  for(auto &nd : nodes)
  {
    // find kNN
    std::vector<int> neighbors;
    nearestNeighbors(nd, neighbors);

    for(const auto neighbor_id : neighbors)
    {
      // adds edge from nd to neighbor
      // and from neighbor to nd
      addEdge(nd.id, neighbor_id);

    } // end inner loop
  } // end outer loop


}


void RoadMap::nearestNeighbors(const Node &query, std::vector<int> &neighbors)
{
  // // TODO: find better method than searching all neighbors i.e kd trees

  neighbors.reserve(k);

  // map distance to node index
  // the size is size-1 b/c we do not want to compose distance to self
  std::unordered_map<double, unsigned int> umap(nodes.size()-1);
  std::vector<double> distances(nodes.size()-1);


  // std::cout << "------------" << std::endl;
  // std::cout << "query id: " << query.id <<  std::endl;

  unsigned int i = 0;
  for(const auto &nd : nodes)
  {
    // do not compute distance to self
    if (query.id == nd.id)
    {
      continue;
    }


    const auto d = euclideanDistance(nd.point.x, nd.point.y, query.point.x, query.point.y);
    distances.at(i) = d;
    umap.insert({d, nd.id});

    // std::cout << nd.point << " id: " << nd.id << " distance: " << d <<  std::endl;

    i++;
  } // end loop


  // sort in descending order
  std::sort(distances.begin(), distances.end(), std::greater<double>());

  // pop off kth NN
  for(unsigned int i = 0; i < k; i++)
  {
    neighbors.push_back(umap.at(distances.back()));
    // std::cout << umap.at(distances.back()) << std::endl;
    distances.pop_back();
  } // end loop

}


void RoadMap::addNode(const Vector2D &q)
{
  Node nd;
  nd.id = nodes.size();
  nd.point = q;
  nodes.push_back(nd);
}


void RoadMap::addEdge(int id1, int id2)
{
  // make sure not to add edge to self
  if (id1 == id2)
  {
    return;
  }

  // distance between nodes
  const auto d = euclideanDistance(nodes.at(id1).point.x, nodes.at(id1).point.y, \
                                   nodes.at(id2).point.x, nodes.at(id2).point.y);


  // check if edge exists between id1 -> id2
  if (!nodes.at(id1).edgeExists(id2))
  {
    Edge edge12;
    edge12.id = id2;
    edge12.d = d;

    nodes.at(id1).edges.push_back(edge12);
    nodes.at(id1).id_set.insert(id2);
  }


  // check if edge exists between id2 -> id1
  if (!nodes.at(id2).edgeExists(id1))
  {
    Edge edge21;
    edge21.id = id1;
    edge21.d = d;

    nodes.at(id2).edges.push_back(edge21);
    nodes.at(id2).id_set.insert(id1);
  }
}





Vector2D RoadMap::randomPoint()
{
  Vector2D v;
  v.x = sampleUniformDistribution(xmin, xmax);
  v.y = sampleUniformDistribution(ymin, ymax);
  return v;
}












} // end namespace
