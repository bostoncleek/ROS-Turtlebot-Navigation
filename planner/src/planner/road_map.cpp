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

  // vector from p1 to p2 (v)
  const auto vx = p2.x - p1.x;
  const auto vy = p2.y - p1.y;

  // leftward normal vector (u)
  const auto ux = -vy;
  const auto uy = vx;

  // magnitude of u
  const auto unorm = std::sqrt(ux*ux + uy*uy);

  // leftward normal vector
  const auto nx = ux / unorm;
  const auto ny = uy / unorm;

  // vector from p1 to p3
  const auto dx = p3.x - p1.x;
  const auto dy = p3.y - p1.y;

  // signed distance is the dot product (d and n)
  return dx*nx + dy*ny;
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

  // Vector2D q1(2.0, 2.8);
  // Vector2D q2(2.0, 1.0);
  //
  // // Vector2D q2(0.2, 0.4);
  // // Vector2D q1(0.6, 1.2);
  //
  // // for(int i = 0; i < obs_map.size(); i++)
  // // {
  // //   bool status = lnSegIntersectPolygon(obs_map.at(i), q1, q2);
  // //   std::cout << "Poly: " << i << "| intersection: " << status << std::endl;
  // // }
  //
  // bool status = lnSegIntersectPolygon(obs_map.at(2), q1, q2);
  // std::cout << "intersection: " << status << std::endl;
  //
  //
  //
  // // bool status = straightLinePath(q1, q2);
  // // std::cout << "path can go: " << status << std::endl;
  //
  // addNode(q1);
  // addNode(q2);
  //
  // addEdge(0,1);

  // clear prvious graph
  nodes.clear();

  while(nodes.size() < n)
  {
    Vector2D q = randomPoint();

    if(!collideWalls(q) and isFreeSpace(q))
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
      if(straightLinePath(nd.point, nodes.at(neighbor_id).point))
      {
        addEdge(nd.id, neighbor_id);
      }
    } // end inner loop
  } // end outer loop


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
    // is not Cfree
    if (ptInsidePolygon(poly, q))
    {
      // std::cout << "inside polygon" << std::endl;
      return false;
    }
  } // end  outer loop

  // std::cout << "outisde polygons" << std::endl;
  // is Cfree
  return true;
}


bool RoadMap::straightLinePath(const Vector2D &p1, const Vector2D &p2) const
{
  int i = 0;

  std::cout << "---------------------" << std::endl;
  for(const auto &poly : obs_map)
  {
    std::cout << "Checking Polygon: " << i << std::endl;

    if (lnSegIntersectPolygon(poly, p1, p2))
    {
      // std::cout << "inside polygon" << std::endl;
      return false;
    }

    i++;
  } // end  outer loop
  std::cout << "---------------------" << std::endl;

  // std::cout << "outisde polygons" << std::endl;
  return true;
}




bool RoadMap::ptInsidePolygon(const polygon &poly, const Vector2D &q) const
{
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


    // just need to be on the right side of one edge and we are good
    if (dist < 0.0 and !rigid2d::almost_equal(dist, 0.0))
    {

      // if on the segment the edge exists make sure it is
      // outside the threshold
      auto dfs = 0.0;
      if (minDistLineSegPt(dfs, p1, p2, q))
      {
        if (dfs > bnd_rad)
        {
          // std::cout << "safe range: " << dfs << std::endl;
          // flag = false;
          return false;
        }

        // near edge segment
        else
        {
          // std::cout << "too close: " << dfs << std::endl;
          return true;
        }
      }

      // min distance is not on the edge segment
      // check distance from ends of segment to
      // ensure q is not near
      else
      {
        const auto d1 = euclideanDistance(p1.x, p1.y, q.x, q.y);
        const auto d2 = euclideanDistance(p2.x, p2.y, q.x, q.y);

        if((d1 > bnd_rad) and (d2 > bnd_rad))
        {
          // std::cout << "not on segment and far away" << std::endl;
          return false;
        }

        else
        {
          // std::cout << "not on segment but still to close" << std::endl;
          return true;
        }
      }
    }

  } // end loop

  return true;
}


bool RoadMap::lnSegIntersectPolygon(const polygon &poly,
                                    const Vector2D &p1,
                                    const Vector2D &p2) const
{
  // http://geomalgorithms.com/a13-_intersect-4.html

  // TODO: check p1 == p2

  auto tE = 0.0;                    // the maximum entering segment parameter
  auto tL = 1.0;                    // the minimum leaving segment parameter
  auto t = 0.0, N = 0.0, D = 0.0;   // intersect parameter t = N / D

  const Vector2D dS = p2 - p1;             // line segment from p1 -> p2


  for(unsigned int i = 0; i < poly.size(); i++)
  {
    std::cout << "----------------------" << std::endl;

    std::cout << "checking edge: " << i << std::endl;

    // vertices on polygon (CCW)
    Vector2D v1, v2;

    if (i != poly.size()-1)
    {
      v1 = poly.at(i);
      v2 = poly.at(i+1);
    }

    // at last vertex, compare with first to get edge
    else
    {
      v1 = poly.back();
      v2 = poly.at(0);
    }

    // edge v1 => v2
    const Vector2D e = v2 - v1;
    // const auto ex = v2.x - v1.x;
    // const auto ey = v2.y - v1.y;

    // outward normal to edge
    const Vector2D ne(e.y, -e.x);
    // const auto nex = ey;
    // const auto ney = -ex;

    // vector from v1 => p1
    const Vector2D pv = p1 - v1;
    // const auto pvx = p1.x - v1.x;
    // const auto pvy = p1.y - v1.y;

    // -dot((P1-V1), ne)
    N = -(pv.x*ne.x + pv.y*ne.y);
    // N = -(pvx*nex + pvy*ney);

    // dot(dS, ne)
    D = dS.x*ne.x + dS.y*ne.y;
    // D = dS.x*nex + dS.y*ney;

    // time of intersection if it happend
    t = N / D;

    // dS is parallel to e
    if (rigid2d::almost_equal(D, 0.0))
    {
      // p1 is outside e
      // therefore so is dS
      if (N < 0.0)
      {
        std::cout << "ds parallel to e and p1 is outside of e" << std::endl;
        std::cout << "No I" << std::endl;
        return false;
      }

      // keep cheking the other edges
      else
      {
        std::cout << "check the other edges" << std::endl;
        continue;
      }
    }

    // dS enters polygon
    if (D < 0.0)
    {
      std::cout << "enters" << std::endl;
      if (t > tE)
      {
        tE = t;
        std::cout << "tE = t" << std::endl;
        // enters after leaving, not possible
        if (tE > tL)
        {
          std::cout << "enters after leaving" << std::endl;
          std::cout << "No I" << std::endl;
          return false;
        }
      }
    }

    // dS is leaving polygon
    else
    {
      std::cout << "leaves" << std::endl;
      if (t < tL)
      {
        tL = t;
        std::cout << "tL = t" << std::endl;
        // leaves before entering, not possible
        if (tL < tE)
        {
          std::cout << "leaves before entering" << std::endl;
          std::cout << "No I" << std::endl;
          return false;
        }
      }
    }
    std::cout << "----------------------" << std::endl;
  } // end loop


  std::cout << "I !!" << std::endl;

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
