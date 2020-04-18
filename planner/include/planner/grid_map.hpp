#ifndef GRID_PLANNER_GUARD_HPP
#define GRID_PLANNER_GUARD_HPP
// \file
/// \brief Creates an 2D grid for planning

#include <cmath>
#include <iosfwd>
#include <vector>

#include "planner/road_map.hpp"


namespace planner
{

  using rigid2d::Vector2D;
  using rigid2d::euclideanDistance;

  /// \brief coordinated in the grid
  struct GridCoordinates
  {
    int i = -1; // row
    int j = -1; // column
  };


  /// \brief Grid cell
  struct Cell
  {
    int state;        // occupied: 1, unoccupied: 0, inflation: 2, unknown: -1
    int i, j;         // index location in grid

    Vector2D p;       // (x,y) world location

    /// \brief default values
    Cell(): state(-1), i(-1), j(-1) {}

    /// \brief set cell values
    /// \param st - state of cell
    Cell(int st, int row, int col): state(st), i(row), j(col) {}
  };


  // TODO: move this to utilities
  //       repeat definition in bmapping
  /// \brief Dimensions of grid
  /// \param lower - lower limit
  /// \param upper - upper limit
  /// \param resolution - resolution of grid
  /// \returns size of grid
  unsigned int gridSize(double lower, double upper, double resolution);

  /// \brief Compose the bounding radius for determining obstacle cells
  /// \param inflation - obstalce inflation distance
  /// \param resolution - length of a grid cell edge (all edges are equal)
  /// \returns the bounding radius
  double boundingRad(double inflation, double resolution);


  /// \brief Constructs an 2D grid
  class GridMap
  {
  public:
    /// \brief 2D grid parameters
    /// \param xmin - x low bound of grid
    /// \param xmax - x upper bound of grid
    /// \param ymin - y low bound of grid
    /// \param ymax - y upper bound of grid
    /// \param resolution - grid resolution
    /// \param inflation - obstacle inflation radius
    /// \param obs_map - coordinates of all polygons in Cspace
    GridMap(double xmin, double xmax,
            double ymin, double ymax,
            double resolution, double inflation,
            obstacle_map obs_map);

    /// \brief Compose a map viewable in rviz
    /// map[out] a map in row major order
    void getGrid(std::vector<int8_t> &map) const;

    /// \brief Determines state of all cells in grid
    void labelCells();

  private:
    /// \brief Labels all osbtacle cells
    /// \param poly - plygon to examine
    /// cell[out] - labels if it is an osbtacle cell
    void collisionCells(const polygon &poly, Cell &cell);

    /// \brief Check if cell collides with boundaries of map
    /// cell[out] - labels if it is an obstacle cell
    void collideWalls(Cell &cell);

    /// \brief Coverts the real-world coordinates of the obstacles into
    ///        grid cell coordinates. Finds the (x,y) grid coordinates
    ///        of a cell's center that correspond to the vertex of an obstacle.
    void preProcessObstacles();

    /// \brief Coverts the real-world coordinates of the boundaries into
    ///        grid cell coordinates. Finds the (x,y) grid coordinates
    ///        of a cell's center that correspond to the boundary.
    // void preProcessMapBounds();


    /// \brief Converts grid indices to word coordinates (x, y)
    /// \param i - row in the grid
    /// \param j - column in the grid
    /// \returns word coordinates
    Vector2D grid2World(int i, int j) const;

    /// \brief Converts floating point world coordinates to grid coordinates
    /// \param x - x position in world
    /// \param y - y position in world
    /// \returns the grid index in row major order
    GridCoordinates world2Grid(double x, double y) const;

    /// \brief Converts grid indices to row major oreder index
    /// \param i - row in the grid
    /// \param j - column in the grid
    /// \returns the grid index in row major order
    unsigned int grid2RowMajor(int i, int j) const;

    double xmin, xmax, ymin, ymax;               // map dims
    double resolution;                           // map resolution
    double bnd_rad;                              // obstacle inflation radius
    obstacle_map obs_map;                        // collection of all the polygons

    int xsize, ysize;                            // number of discretization
    std::vector<Cell> grid;                      // 2D grid of Cspace


  };


  /// \brief output a 2 dimensional grid coordinates
  /// os - stream to output to
  /// gc - grid coordinates
  std::ostream & operator<<(std::ostream & os, const GridCoordinates & gc);



} // end namespace

#endif
