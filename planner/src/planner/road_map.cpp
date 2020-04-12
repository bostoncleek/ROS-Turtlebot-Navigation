/// \file
/// \brief Probabilisitc road maps

#include <stdexcept>
#include <cmath>
#include <iostream>

#include "planner/road_map.hpp"


namespace planner
{

bool minDistLineSegPt(double &distance,
                      const Vector2D &p1,
                      const Vector2D &p2,
                      const Vector2D &p3)
{
  // source http://paulbourke.net/geometry/pointlineplane/

  const auto dx = p2.x - p1.x;
  const auto dy = p2.y - p1.y;

  const auto num = (p3.x - p1.x)*dx + (p3.y - p1.y)*dy;
  const auto denom = dx*dx + dy*dy;
  const auto u = num / denom;

  // std::cout << u << std::endl;

  // location of point (P) on line (not necessarily on the line segment)
  const auto px = p1.x + u * dx;
  const auto py = p1.y + u * dy;

  // min distance
  distance = euclideanDistance(px, py, p3.x, p3.y);

  if (rigid2d::almost_equal(u, 0.0) or rigid2d::almost_equal(u, 1.0))
  {
    return true;
  }

  else if (u > 0.0 and u < 1.0)
  {
    return true;
  }

  // return (u >= 0.0 and u <= 1.0) ? true : false;
  return false;
}


double minDistLineSegPt(const Vector2D &p1,
                        const Vector2D &p2,
                        const Vector2D &p3)
{
  return (p3.x - p1.x)*(p2.y - p1.y) - (p3.y - p1.y)*(p2.x - p1.x);
  // return (p2.x - p1.x)*(p3.y - p1.y) - (p2.y - p1.y)*(p3.x - p1.x);
}


double minDist(const Vector2D &p1,
                        const Vector2D &p2,
                        const Vector2D &p3)
{
  const auto dx = p2.x - p1.x;
  const auto dy = p2.y - p1.y;

  const auto num = std::fabs(dy*p3.x - dx*p3.y + p2.x*p1.y - p2.y*p1.x);
  const auto denom = std::sqrt(dx*dx + dy*dy);

  return num / denom;
}



RoadMap::RoadMap(double xmin, double xmax,
                 double ymin, double ymax,
                 double bnd_rad,
                 unsigned int neighbors, unsigned int num_nodes,
                 obstacle_map obs_map)
                    : xmin(xmin),
                      xmax(xmax),
                      ymin(ymin),
                      ymax(ymax),
                      bnd_rad(bnd_rad),
                      k(neighbors),
                      n(num_nodes),
                      obs_map(obs_map)
{
  if (n <= k)
  {
    throw std::invalid_argument("Number of nodes in road map less than nearest neighbors");
  }
}


void RoadMap::getRoadMap(std::vector<Node> &roadmap) const
{
  roadmap = nodes;
}


void RoadMap::printRoadMap() const
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



void RoadMap::constructRoadMap(const Vector2D &start, const Vector2D &goal)
{

  // TODO: check if nodes are away from perimater walls


  // clear prvious graph
  nodes.clear();

  // add nodes
  while(nodes.size() < n)
  {
    Vector2D q = randomPoint();

    if(!collideWalls(q) and isFreeSpace(q))
    {
      addNode(q);
    }
  } // end while loop


  // add edges
  // for(auto &nd : nodes)
  // {
  //   // find kNN
  //   std::vector<int> neighbors;
  //   nearestNeighbors(nd, neighbors);
  //
  //   for(const auto neighbor_id : neighbors)
  //   {
  //     // adds edge from nd to neighbor
  //     // and from neighbor to nd
  //     addEdge(nd.id, neighbor_id);
  //
  //   } // end inner loop
  // } // end outer loop


}


void RoadMap::nearestNeighbors(const Node &query, std::vector<int> &neighbors) const
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


bool RoadMap::isFreeSpace(const Vector2D &q) const
{
  // check if q is on the left side of all edges
  // for a polygon
  // if on the right side check if it is within
  // the bounding radius theshold

  for(const auto &poly : obs_map)
  {
    // std::cout << "---------------------" << std::endl;

    // loop over all vertices
    bool collision = true;
    for(unsigned int i = 0; i < poly.size(); i++)
    {
      Vector2D p1, p2;

      if (i != poly.size()-1)
      {
        p1 = poly.at(i);
        p2 = poly.at(i+1);
      }

      // at last vertex, compare with first to get edge
      else
      {
        p1 = poly.back();
        p2 = poly.at(0);
      }


      // min distance to line
      auto dist = minDistLineSegPt(p1, p2, q);
      // std::cout << dist << std::endl;

      auto dist2 = 0.0;
      minDistLineSegPt(dist2, p1, p2, q);

      auto dist3 = minDist(p1, p2, q);

      std::cout << dist << " " << dist2 << " " << dist3 << std::endl;



      // if (minDistLineSegPt(dist2, p1, p2, q))
      // {
      //   std::cout << dist << " " << dist2 << std::endl;
      // }



      // // right side is outisde of polygon
      if (dist > 0.0)
      {
        // check that q is outside radius threshold
        // distance from segment to q
        // also checking that this distance is on the segment
        // auto dfs = 0.0;
        // if (minDistLineSegPt(dfs, p1, p2, q) and dfs < bnd_rad)
        // if (dist < bnd_rad)
        // {
        //   collision = true;
        // }
        //
        // else
        // {
          collision = false;
        // }
      }



    } // end inner loop

    if (collision)
    {
      std::cout << "inside polygon" << std::endl;
      return false;
    }

    // std::cout << "---------------------" << std::endl;


  } // end  outer loop

  std::cout << "outisde polygons" << std::endl;

  return true;
}



bool RoadMap::collideWalls(const Vector2D &q) const
{
  Vector2D v1(xmin, ymin);
  Vector2D v2(xmax, ymin);
  Vector2D v3(xmax, ymax);
  Vector2D v4(xmin, ymax);

  std::vector<Vector2D> bounds = {v1, v2, v3, v4, v1};

  for(unsigned int i = 0; i < bounds.size()-1; i++)
  {
    auto dist = 0.0;
    if (minDistLineSegPt(dist, bounds.at(i), bounds.at(i+1), q) and dist < bnd_rad)
    {
      // std::cout << "node to close to wall" << std::endl;
      return true;
    }
  } // end loop

  return false;
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


Vector2D RoadMap::randomPoint() const
{
  Vector2D v;
  v.x = sampleUniformDistribution(xmin, xmax);
  v.y = sampleUniformDistribution(ymin, ymax);
  return v;
}












} // end namespace
