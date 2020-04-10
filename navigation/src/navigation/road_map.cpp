/// \file
/// \brief Probabilisitc road maps

#include <stdexcept>
#include <cmath>
#include <iostream>


#include "navigation/road_map.hpp"

namespace navigation
{

// TODO: min dist threshold for KNN


RoadMap::RoadMap(double xmin, double xmax,
                 double ymin, double ymax,
                 double obs_dist, double node_dist,
                 unsigned int neighbors, unsigned int num_nodes)
                    : xmin(xmin),
                      xmax(xmax),
                      ymin(ymin),
                      ymax(ymax),
                      obs_dist(obs_dist),
                      node_dist(node_dist),
                      k(neighbors),
                      n(num_nodes)
{
  if (n <= k)
  {
    throw std::invalid_argument("Number of nodes in road map less than nearest neighbors");
  }
}



void RoadMap::constructRoadMap(const FeatureMap &map, const Vector2D &start, const Vector2D &goal)
{
  // TODO: // check if distance is greater than theshold for edges



  // clear prvious graph
  nodes.clear();

  // add nodes
  while(nodes.size() < n)
  {
    Vector2D q = randomPoint();

    if(isFreeSpace(map, q))
    {
      Node nd;
      nd.id = nodes.size();
      nd.point = q;

      nodes.push_back(nd);
    }
  }

  // add edges
  for(auto &nd : nodes)
  {
    // find kNN
    std::vector<int> neighbors;
    nearestNeighbors(nd, neighbors);

    for(const auto id : neighbors)
    {
      // check if edge exists and
      // check collisions
      if (!edgeCollision(map, nd.point, nodes.at(id).point))
      {
        // distance between nodes
        const auto d = euclideanDistance(nd.point.x, nd.point.y, nodes.at(id).point.x, nodes.at(id).point.y);

        // connect nd to id of node
        if (!nd.edgeExists(id))
        {
          Edge edge1;
          edge1.next_id = id;
          edge1.d = d;

          nd.edges.push_back(edge1);
          nd.id_set.insert(id);
        }

        // connect id of node to nd
        if (!nodes.at(id).edgeExists(nd.id))
        {
          Edge edge2;
          edge2.next_id = nd.id;
          edge2.d = d;

          nodes.at(id).edges.push_back(edge2);
          nodes.at(id).id_set.insert(nd.id);
        }
      }

    } // end inner loop
  } // end outer loop


}

void RoadMap::printRoadMap()
{
  for(const auto &nd : nodes)
  {
    std::cout << nd.id << "| ";

    for(const auto &ed : nd.edges)
    {
      // std::cout << "id: " <<  ed.next_id << " d: " << ed.d << " ";
      std::cout << "id: " <<  ed.next_id << " ";
    }
    std::cout << std::endl;
  }

}





void RoadMap::nearestNeighbors(const Node &query, std::vector<int> &neighbors)
{
  // TODO: find better method than searching all neighbors i.e kd trees

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
  }


  // sort in descending order
  std::sort(distances.begin(), distances.end(), std::greater<double>());

  // pop off k NN
  for(unsigned int i = 0; i < k; i++)
  {
    neighbors.push_back(umap.at(distances.back()));
    // std::cout << umap.at(distances.back()) << std::endl;
    distances.pop_back();
  }

}




bool RoadMap::isFreeSpace(const FeatureMap &map, const Vector2D &q)
{
  // TODO: find better method than searching all obstacle i.e kd trees

  for(unsigned int i = 0; i < map.cx.size(); i++)
  {
    const auto d = euclideanDistance(map.cx.at(i), map.cy.at(i), q.x, q.y);

    // collision found
    if (d < map.r.at(i) + obs_dist)
    {
      std::cout << "Not free space"  <<  std::endl;

      return false;
    }
  }

  return true;
}



bool RoadMap::edgeCollision(const FeatureMap &map, const Vector2D &p1, const Vector2D &p2)
{
  // source http://paulbourke.net/geometry/pointlineplane/

  for(unsigned int i = 0; i < map.cx.size(); i++)
  {
    const auto dx = p2.x - p1.x;
    const auto dy = p2.y - p1.y;


    const auto num = (map.cx.at(i) - p1.x) * dx + (map.cy.at(i) - p1.y) * dy;
    const auto denom = dx * dx + dy * dy;
    const auto u = num / denom;


    if (u >= 0 and u <= 1)
    {
      // location of point (P) on line segment
      const auto Px = p1.x + u * dx;
      const auto Py = p1.y + u * dy;

      // distance from P to center of landmark
      const auto d = euclideanDistance(map.cx.at(i), map.cy.at(i), Px, Py);

      if (d < map.r.at(i) + obs_dist)
      {
        std::cout << "Edge collision"  <<  std::endl;
        return true;
      }
    }
  }

  return false;
}




Vector2D RoadMap::randomPoint()
{
  Vector2D v;
  v.x = sampleUniformDistribution(xmin, xmax);
  v.y = sampleUniformDistribution(ymin, ymax);
  return v;
}


// void RoadMap::addNode()
// {
//
//
// }


} // end namespace
