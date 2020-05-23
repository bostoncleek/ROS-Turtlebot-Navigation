/// \file
/// \brief Probabilisitc road maps

#include <stdexcept>
#include <cmath>
#include <iostream>
#include <unordered_map>

#include "planner/road_map.hpp"


namespace planner
{


bool lnSegIntersectPolygon(const polygon &poly,
                           const Vector2D &p1,
                           const Vector2D &p2)
{
  // http://geomalgorithms.com/a13-_intersect-4.html

  // check p1 == p2
  // dp1p2 = euclideanDistance(p1.x, p1.y, p2.x, p2.y);
  // if (rigid2d::almost_equal(dp1p2, 0.0))
  // {
  //   return false;
  // }

  auto tE = 0.0;                    // the maximum entering segment parameter
  auto tL = 1.0;                    // the minimum leaving segment parameter
  auto t = 0.0, N = 0.0, D = 0.0;   // intersect parameter t = N / D

  const Vector2D dS = p2 - p1;             // line segment from p1 -> p2


  for(unsigned int i = 0; i < poly.size(); i++)
  {
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

    // outward normal to edge
    const Vector2D ne(e.y, -e.x);

    // vector from v1 => p1
    const Vector2D pv = p1 - v1;

    // -dot((P1-V1), ne)
    N = -(pv.x*ne.x + pv.y*ne.y);

    // dot(dS, ne)
    D = dS.x*ne.x + dS.y*ne.y;

    // time of intersection if it happend
    t = N / D;

    // dS is parallel to e
    if (rigid2d::almost_equal(D, 0.0))
    {
      // p1 is outside e
      // therefore so is dS
      if (N < 0.0)
      {
        return false;
      }

      // keep cheking the other edges
      else
      {
        continue;
      }
    }

    // dS enters polygon
    if (D < 0.0)
    {
      if (t > tE)
      {
        tE = t;
        // enters after leaving, not possible
        if (tE > tL)
        {
          return false;
        }
      }
    }

    // dS is leaving polygon
    else
    {
      if (t < tL)
      {
        tL = t;
        // leaves before entering, not possible
        if (tL < tE)
        {
          return false;
        }
      }
    }
  } // end loop

  return true;
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
  // TODO: add check to make sure start/goal are within bounds

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
  // clear prvious graph
  nodes.clear();

  if (!isFreeSpace(start) and !collideWalls(start))
  {
    std::cout << "ERROR: starting position NOT valid" << std::endl;
    return;
  }

  if (!isFreeSpace(goal) and !collideWalls(goal))
  {
    std::cout << "ERROR: goal position NOT valid" << std::endl;
    return;
  }

  // start adding nodes
  while(nodes.size() < n)
  {
    Vector2D q = randomPoint();
    // Vector2D q = hc.at(i);

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
      if(stlnPathCollision(nd.point, nodes.at(neighbor_id).point))
      {
        addEdge(nd.id, neighbor_id);
      }
    } // end inner loop
  } // end outer loop

  // add start and end goals
  if(!addStartGoalConfig(start, goal))
  {
    std::cout << "ERROR: Disconnected graph" << std::endl;
  }

}


bool RoadMap::stlnPathCollision(const Vector2D &p1, const Vector2D &p2) const
{
  for(const auto &poly : obs_map)
  {
    if (lnSegIntersectPolygon(poly, p1, p2) or lnSegClose2Polygon(poly, p1, p2))
    {
      return false;
    }
  } // end  outer loop

  return true;
}


bool RoadMap::addStartGoalConfig(const Vector2D &start, const Vector2D &goal)
{
  const auto start_id = addNode(start);
  const auto goal_id = addNode(goal);


  // KNN for start
  std::vector<int> neighbors;
  nearestNeighbors(nodes.at(start_id), neighbors);

  for(const auto neighbor_id : neighbors)
  {
      // adds edge from nd to neighbor
      // and from neighbor to nd
      if(stlnPathCollision(nodes.at(start_id).point, nodes.at(neighbor_id).point))
      {
        addEdge(nodes.at(start_id).id, neighbor_id);
      }
  }

  // start node has neighbors
  if (nodes.at(start_id).edges.empty())
  {
    std::cout << "ERROR: start node NOT connected to PRM" << std::endl;
    return false;
  }

  // KNN for goal
  neighbors.clear();
  nearestNeighbors(nodes.at(goal_id), neighbors);

  for(const auto neighbor_id : neighbors)
  {
      // adds edge from nd to neighbor
      // and from neighbor to nd
      if(stlnPathCollision(nodes.at(goal_id).point, nodes.at(neighbor_id).point))
      {
        addEdge(nodes.at(goal_id).id, neighbor_id);
      }
  }

  // goal node has neighbors
  if (nodes.at(goal_id).edges.empty())
  {
    std::cout << "ERROR: goal node NOT connected to PRM" << std::endl;
    return false;
  }

  return true;
}





void RoadMap::nearestNeighbors(const Node &query, std::vector<int> &neighbors) const
{
  // // TODO: find better method than searching all neighbors i.e kd trees
  neighbors.reserve(k);

  // map distance to node index
  // the size is size-1 b/c we do not want to compose distance to self
  std::unordered_map<double, unsigned int> umap(nodes.size()-1);
  std::vector<double> distances(nodes.size()-1);

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
      return true;
    }
  } // end loop

  return false;
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
      return false;
    }
  } // end  outer loop
  // is Cfree
  return true;
}



bool RoadMap::ptInsidePolygon(const polygon &poly, const Vector2D &q) const
{
  for(unsigned int i = 0; i < poly.size(); i++)
  {
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

    // min distance to line
    ClosePoint clpt = signMinDist2Line(v1, v2, q);


    // on right side of edge
    if (clpt.sign_d < 0.0 and !rigid2d::almost_equal(clpt.sign_d, 0.0))
    {
      // if on the segment the edge exists make sure it is
      // outside the threshold
      if (clpt.on_seg)
      {
        if (std::fabs(clpt.sign_d) > bnd_rad)
        {
          return false;
        }

        else
        {
          return true;
        }
      }

      // min distance is not on the edge segment
      // check distance from ends of segment to
      // ensure q is not near
      else
      {
        // check which side of the polygon edge we are on (v1 or v2)
        // to the left of v1
        // compare distance from v1 to q
        if (clpt.t < 0.0)
        {
          const auto dv1p = euclideanDistance(v1.x, v1.y, q.x, q.y);
          if (dv1p > bnd_rad)
          {
            return false ;
          }

          else
          {
            return true;
          }
        }

        // to the left of v2
        // compare distance from v2 to q
        else if (clpt.t > 1.0)
        {
          const auto dv2p = euclideanDistance(v2.x, v2.y, q.x, q.y);
          if (dv2p > bnd_rad)
          {
            return false;
          }

          else
          {
            return true;
          }
        }
      }

    } // end if (right side)
  } // end loop

  return true;
}


bool RoadMap::lnSegClose2Polygon(const polygon &poly,
                        const Vector2D &p1,
                        const Vector2D &p2) const
{
  // case 1: check dist from corners (p3) of polygon to p1 => p2

  // NOTE: case 2 is covered in the free space search
  // case 2: either one or maybe both of the bounds of the line segment
  //         p1  and/or p2 are the closest points to the edge v1 => v2
  //         of the polygon

  for(unsigned int i = 0; i < poly.size(); i++)
  {
    // vertices on polygon (CCW)
    Vector2D v1, v2;

    if (i != poly.size()-1)
    {
      v1 = poly.at(i);
      v2 = poly.at(i+1);
    }

    else
    {
      v1 = poly.back();
      v2 = poly.at(0);
    }

    // case 1:
    ClosePoint clpt = signMinDist2Line(p1, p2, v1);

    // on line segment p1 => p2 and within radius there is a collision
    if (clpt.on_seg and std::fabs(clpt.sign_d) < bnd_rad)
    {
      // std::cout << "corner of polygon" << std::endl;
      return true;
    }

    // // case 2:
    // // check p1: closest point on edge v1 => v2 to p1
    // ClosePoint clpt_p1 = signMinDist2Line(v1, v2, p1);
    //
    // if (clpt_p1.on_seg and std::fabs(clpt_p1.sign_d) < bnd_rad)
    // {
    //   std::cout << "p1 too close" << std::endl;
    //   return true;
    // }
    //
    // // check p2: closest point on edge v1 => v2 to p2
    // ClosePoint clpt_p2 = signMinDist2Line(v1, v2, p2);
    //
    // if (clpt_p2.on_seg and std::fabs(clpt_p2.sign_d) < bnd_rad)
    // {
    //   std::cout << "p2 too close" << std::endl;
    //   return true;
    // }
  } // end loop

  return false;
}


int RoadMap::addNode(const Vector2D &q)
{
  Node nd;
  nd.id = nodes.size();
  nd.point = q;
  nodes.push_back(nd);
  return nd.id;
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
