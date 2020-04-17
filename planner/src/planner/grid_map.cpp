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
}


void GridMap::getGrid(std::vector<int8_t> &map) const
{
  // IMPORTANT
  // Transpose map
  map.resize(grid.size());

  for(unsigned int i = 0; i < grid.size(); i++)
  {
    auto row = i / xsize;
    auto col = i % xsize;
    auto idx = col * xsize + row;


    if (grid.at(i).state == 1)
    {
      map.at(idx) = 255;
    }

    else if (grid.at(i).state == 0)
    {
      map.at(idx) = 0;
    }

    else
    {
      map.at(i) = -1;
    }

  }
}


Vector2D GridMap::grid2World(int i, int j) const
{
  if (!(i >= 0 and i <= xsize-1))
  {
    throw std::invalid_argument("ith row out of grid bounds");
  }

  if (!(j >= 0 and j <= ysize-1))
  {
    throw std::invalid_argument("jth column out of grid bounds");
  }

  Vector2D w;
  w.x = i * resolution + resolution/2.0 + xmin;
  w.y = j * resolution + resolution/2.0 + ymin;

  return w;
}





unsigned int GridMap::world2RowMajor(double x, double y) const
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
  }


  auto j = std::floor((y - ymin) / resolution);
  // Case: y query == 0 and ymin < 0
  //     or y query == ysize and ymin == 0
  if (j == ysize)
  {
    j--;
  }

  auto index = grid2RowMajor(i, j);
  return index;
}



unsigned int GridMap::grid2RowMajor(int i, int j) const
{
  // IMPORTANT: swap i and j because rviz uses rows for y-axis
  //            and col for x-axis
  return i * xsize + j;
}






















} // end namespace
