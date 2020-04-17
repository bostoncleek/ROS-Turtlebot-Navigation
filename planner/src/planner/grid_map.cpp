// \file
/// \brief Creates an occupancy grid

#include <iostream>

#include "planner/grid_map.hpp"

namespace planner
{

unsigned int gridSize(double lower, double upper, double resolution)
{
  return static_cast<unsigned int> (std::ceil((upper - lower) / resolution));
}


GridMap::GridMap(double xmin, double xmax,
                 double ymin, double ymax,
                 double resolution, double inflation,
                 obstacle_map obs_map)
                  : xmin(xmin),
                    xmax(xmax),
                    ymin(ymin),
                    ymax(ymax),
                    resolution(resolution),
                    inflation(inflation),
                    obs_map(obs_map),
                    xsize(gridSize(xmin, xmax, resolution)),
                    ysize(gridSize(ymin, ymax, resolution)),
                    grid(xsize * ysize)
{
  preProcessObstacles(obs_map);
}


void GridMap::getGrid(std::vector<int8_t> &map) const
{
  // IMPORTANT
  // Transpose map
  map.resize(grid.size());

  for(unsigned int i = 0; i < grid.size(); i++)
  {
    auto row = i / ysize;
    auto col = i % ysize;

    auto idx = col * xsize + row;


    if (grid.at(i).state == 1)
    {
      map.at(idx) = 100;
    }

    else if (grid.at(i).state == 0)
    {
      map.at(idx) = 0;
    }

    else
    {
      map.at(idx) = -1;
    }

  }
}


void GridMap::preProcessObstacles(obstacle_map &obs_map)
{

  // std::cout << grid.size() << std::endl;
  // std::cout << xsize << std::endl;
  // std::cout << ysize << std::endl;

  // loop across all obstacles
  for(unsigned int i = 0; i < obs_map.size(); i++)
  // for(unsigned int i = 0; i < 3; i++)
  {
    // loop across all vertices of each obstacle
    for(unsigned int j = 0; j < obs_map.at(i).size(); j++)
    {
      // std::cout << "world (x,y) " << obs_map.at(i).at(j) << std::endl;

      // transform real-world coordinates
      // int0 grid cell center coordinates
      const auto x = obs_map.at(i).at(j).x;
      const auto y = obs_map.at(i).at(j).y;

      // row-major order index in grid
      const GridCoordinates gc = world2Grid(x, y);
      // std::cout << gc << std::endl;

      // IMPORTANT: map is transposed to world
      // convert grid row and col pair into grid (x, y)
      // update the vertex of the obstacle
      obs_map.at(i).at(j) = grid2World(gc.i, gc.j);


      // IMPORTANT: now we can get the grid index
      //            by transposing
      const auto gidx = grid2RowMajor(gc.i, gc.j);
      // const auto gidx = gc.i * ysize + gc.j;

      // grid.at(gidx).state = 1;
      // std::cout << "grid (x,y) " << obs_map.at(i).at(j) << std::endl;

    } // end inner loop
  } // end outer loop
}






Vector2D GridMap::grid2World(int i, int j) const
{
  if (!(i >= 0 and i <= xsize-1))
  {
    std::cout << "i: " << i << std::endl;
    throw std::invalid_argument("ith row out of grid bounds");
  }

  if (!(j >= 0 and j <= ysize-1))
  {
    std::cout << "j: " << j << std::endl;
    throw std::invalid_argument("jth column out of grid bounds");
  }

  Vector2D w;
  w.x = i * resolution + resolution/2.0 + xmin;
  w.y = j * resolution + resolution/2.0 + ymin;

  return w;
}





GridCoordinates GridMap::world2Grid(double x, double y) const
{
  if (!(x >= xmin and x <= xmax))
  {
    throw std::invalid_argument("X position NOT in the bounds of the world");
  }

  if (!(y >= ymin and y <= ymax))
  {
    throw std::invalid_argument("Y position NOT in the bounds of the world");
  }

  // round down to the nearest cell
  // do not want to over estimate the index
  auto i = std::floor((x - xmin) / resolution);
  // Case: x query == 0 and xmin < 0
  //        (0 - (-3)) / 1 = 3 == xsize
  //     or x query == xsize and xmin == 0
  //        (3 - 0) / 1 = 3 == xsize


  if (i == xsize)
  {
    i--;
    // std::cout << "i (dec): " << i << std::endl;
  }


  auto j = std::floor((y - ymin) / resolution);
  // Case: y query == 0 and ymin < 0
  //     or y query == ysize and ymin == 0


  if (j == ysize)
  {
    j--;
    // std::cout << "j (dec): " << j << std::endl;
  }

  GridCoordinates gc;
  gc.i = i;
  gc.j = j;
  return gc;
}



unsigned int GridMap::grid2RowMajor(int i, int j) const
{
  // IMPORTANT: swap i and j because rviz uses rows for y-axis
  //            and col for x-axis
  // std::cout << "i: " << i << std::endl;
  // std::cout << "j: " << j << std::endl;
  // std::cout << "row-major " << i * xsize + j << std::endl;
  // return j * xsize + i;
  return i * ysize + j;
}


std::ostream & operator<<(std::ostream & os, const GridCoordinates & gc)
{
  os << "Grid Coordinates: " <<  "[" << gc.i << " " << gc.j << "]" << "\n";
  return os;
}


} // end namespace
