// \file
/// \brief Creates an occupancy grid

#include <iostream>
#include "planner/grid_map.hpp"

namespace planner
{

unsigned int gridSize(double lower, double upper, double resolution)
{
  return static_cast<unsigned int> (std::round((upper - lower) / resolution));
}


double boundingRad(double inflation, double resolution)
{
  const auto cell_diag = std::sqrt(resolution*resolution);
  return inflation + 0.5 * cell_diag;
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
                    bnd_rad(boundingRad(inflation, resolution)),
                    obs_map(obs_map),
                    xsize(gridSize(xmin, xmax, resolution)),
                    ysize(gridSize(ymin, ymax, resolution)),
                    grid(xsize * ysize)
{
  // convert world (x,y) into grid (x,y), this is the center of the
  // closest grid cell to the obstacle vertex location in the world
  preProcessObstacles();
  
  // std::cout << xsize << " " << ysize << std::endl;
}


void GridMap::getGridViz(std::vector<int8_t> &map) const
{
  // IMPORTANT
  // convert from column to row
  map.resize(grid.size());

  for(unsigned int i = 0; i < grid.size(); i++)
  {
    // convert from column major order to row major order
    // internal map is rotated 90 deg from rviz
    auto row = i / ysize;
    auto col = i % ysize;

    auto idx = col * xsize + row;


    if (grid.at(i).state == 2)
    {
      map.at(idx) = 30;
    }

    else if (grid.at(i).state == 1)
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


void GridMap::getGrid(std::vector<Cell> &grid_cells) const
{
    grid_cells = grid;
}


void GridMap::labelCells()
{
  // loop across all cells
  unsigned int i = 0;
  for(auto &cell : grid)
  {
    // compose location of cell in grid
    auto row = i / ysize;
    auto col = i % ysize;

    cell.i = row;
    cell.j = col;
    cell.id = grid2RowMajor(row, col);

    cell.p = grid2World(row, col);

    // collisions with boundaries of world
    collideWalls(cell);

    // loop across all obstacles
    for(const auto &poly : obs_map)
    // polygon poly = obs_map.at(7);
    {
      collisionCells(poly, cell);

    } // end inner loop

    // label cells as free
    if (cell.state == -1)
    {
      cell.state = 0;
    }

    i++;
  } // end outer loop
}


bool GridMap::worldBounds(int i, int j) const
{
  return ((i >= 0 and i <= xsize-1) and (j >= 0 and j <= ysize-1)) ? true : false;
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


std::vector<int> GridMap::getGridSize() const
{
  std::vector<int> gs = {xsize, ysize};
  return gs;
}


void GridMap::collisionCells(const polygon &poly, Cell &cell)
{
  // flag for determining if cell is inside an obstacle
  bool flag_inside = true;

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

    // v1 = poly.at(0);
    // v2 = poly.at(1);
    //
    // std::cout << "vertex" << std::endl;
    // std::cout << v1 << std::endl;
    // std::cout << v2 << std::endl;



    // min distance to line
    ClosePoint clpt = signMinDist2Line(v1, v2, cell.p);

    // std::cout << "sign d: " << std::fabs(clpt.sign_d) << std::endl;
    // std::cout << "on seg: " << clpt.on_seg << std::endl;
    // std::cout << "t: " << clpt.t << std::endl;


    // on the line and within the segment bounds
    if (rigid2d::almost_equal(clpt.sign_d, 0.0) and clpt.on_seg)
    {
      // std::cout << "on border of obstacle: " << std::fabs(clpt.sign_d) << std::endl;
      cell.state = 1;
      flag_inside = false;
    }


    // edge case: singed distance is zero but not on line segment
    else if (rigid2d::almost_equal(clpt.sign_d, 0.0) and !clpt.on_seg)
    {
      // check which side of the polygon edge we are on (v1 or v2)
      // to the left of v1
      // compare distance from v1 to cell
      if (clpt.t < 0.0)
      {
        const auto dv1p = euclideanDistance(v1.x, v1.y, cell.p.x, cell.p.y);
        if (dv1p > bnd_rad)
        {
          // cell.state = 0;
          flag_inside = false;
        }

        else
        {
          if (cell.state != 1)
          {
            cell.state = 2;
          }
          flag_inside = false;
        }
      }

      else if (clpt.t > 0.0)
      {
        const auto dv2p = euclideanDistance(v2.x, v2.y, cell.p.x, cell.p.y);
        if (dv2p > bnd_rad)
        {
          // cell.state = 0;
          flag_inside = false;
        }

        else
        {
          // std::cout << "here" << std::endl;
          if (cell.state != 1)
          {
            cell.state = 2;
          }
          flag_inside = false;
        }
      }
    }


    // on right side of edge
    else if (clpt.sign_d < 0.0)
    {
      // if on the segment the edge exists make sure it is
      // outside the threshold
      if (clpt.on_seg)
      {
        if (std::fabs(clpt.sign_d) > bnd_rad)
        {
          // cell.state = 0;
          flag_inside = false;
        }

        else
        {
          if (cell.state != 1)
          {
            cell.state = 2;
          }
          flag_inside = false;
        }
      }

      // min distance is not on the edge segment
      // check distance from ends of segment to
      // ensure the cell is not near
      else
      {
        // check which side of the polygon edge we are on (v1 or v2)
        // to the left of v1
        // compare distance from v1 to cell
        if (clpt.t < 0.0)
        {
          const auto dv1p = euclideanDistance(v1.x, v1.y, cell.p.x, cell.p.y);
          if (dv1p > bnd_rad)
          {
            // cell.state = 0;
            flag_inside = false;
          }

          else
          {
            // std::cout << "here" << std::endl;
            if (cell.state != 1)
            {
              cell.state = 2;
            }
            flag_inside = false;
          }
        }

        else if (clpt.t > 0.0)
        {
          const auto dv2p = euclideanDistance(v2.x, v2.y, cell.p.x, cell.p.y);
          if (dv2p > bnd_rad)
          {
            // cell.state = 0;
            flag_inside = false;
          }

          else
          {
            if (cell.state != 1)
            {
              cell.state = 2;
            }
            flag_inside = false;
          }
        }
      }
    } // end if (right side)

  } // end for loop

  // cell is inside an obstacle
  if (flag_inside)
  {
    cell.state = 1;
  }

}


void GridMap::collideWalls(Cell &cell)
{
  // grid (x,y) coordinates of boundary cells
  // lower bounds
  const GridCoordinates gc_min = world2Grid(xmin, ymin);
  const Vector2D grid_min = grid2World(gc_min.i, gc_min.j);

  // upper bounds
  const GridCoordinates gc_max = world2Grid(xmax, ymax);
  const Vector2D grid_max = grid2World(gc_max.i, gc_max.j);


  const Vector2D v1 = grid_min;
  const Vector2D v2(grid_max.x, grid_min.y);
  const Vector2D v3 = grid_max;
  const Vector2D v4(grid_min.x, grid_max.y);

  std::vector<Vector2D> bounds = {v1, v2, v3, v4, v1};

  for(unsigned int i = 0; i < bounds.size()-1; i++)
  {
    // min distance to line
    ClosePoint clpt = signMinDist2Line(bounds.at(i), bounds.at(i+1), cell.p);

    if (rigid2d::almost_equal(clpt.sign_d, 0.0) and clpt.on_seg)
    {
      cell.state = 1;
    }

    else if (std::abs(clpt.sign_d) < bnd_rad and cell.state != 1)
    {
      cell.state = 2;
    }
  }
}


void GridMap::preProcessObstacles()
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

      // IMPORTANT:
      // convert grid row and col pair into grid (x, y)
      // update the vertex of the obstacle
      obs_map.at(i).at(j) = grid2World(gc.i, gc.j);


      // IMPORTANT: now we can get the grid index
      // const auto gidx = grid2RowMajor(gc.i, gc.j);
      // const auto gidx = gc.i * ysize + gc.j;
      // grid.at(gidx).state = 1;

      // if (i == 2)
      // {
      //   std::cout << gc << std::endl;
      //   std::cout << "grid (x,y) " << obs_map.at(i).at(j) << std::endl;
      //
      //   const auto gidx = gc.i * ysize + gc.j;
      //   grid.at(gidx).state = 1;
      //
      // }

    } // end inner loop
  } // end outer loop
}


std::ostream & operator<<(std::ostream & os, const GridCoordinates & gc)
{
  os << "Grid Coordinates: " <<  "[" << gc.i << " " << gc.j << "]" << "\n";
  return os;
}


} // end namespace
