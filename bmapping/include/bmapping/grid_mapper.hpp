#ifndef GRID_GUARD_HPP
#define GRID_GUARD_HPP
/// \file
/// \brief Creates a 2D occupancy grid

#include <cmath>
#include <iosfwd>
#include <vector>
#include <queue>
#include <unordered_set>

#include <rigid2d/rigid2d.hpp>
#include "bmapping/sensor_model.hpp"



namespace bmapping
{
  using rigid2d::Vector2D;
  using rigid2d::Transform2D;
  using rigid2d::TransformData2D;


  /// \brief Convert log odds to a probability
  /// \parma l - log odds
  /// \return probability based on log odd
  constexpr double logOdds2Prob(double l)
  {
    return 1 - (1 / (1 + std::exp(l)));
  }

  /// \brief Convert log odds to a probability
  /// \parma p - probability
  /// \return log odds based on a probability
  constexpr double prob2LogOdds(double p)
  {
    return std::log(p / (1 - p));
  }


  /// \brief probability density function zero mean
  /// \param a - arguement to compute probability of
  /// \param b - variance of the distribution
  /// \returns the likelihood of a
  double pdfNormal(double a, double b);

  /// \brief Dimensions of map
  /// \param lower - lower limit
  /// \param upper - upper limit
  /// \param resolution - resolution of map
  /// \returns size of map
  unsigned int mapSize(double lower, double upper, double resolution);


  /// \brief coordinated in the grid
  struct GridCoordinates
  {
    int i = 0; // column
    int j = 0; // row
  };


  /// \brief a grid cell in the map
  ///        -1 unkown, 0 free, 1 occupied
  struct Cell
  {
    double log_odds;     // log odds
    double prob;         // probability of occupied [0 100]
    double occ_dist;     // distance to nearest occupied cell
    int state;           // occupied state

    int i, j;            // index location in grid
    int src_i, src_j;    // index location of nearest obstacle in grid


    /// \brief default values
    Cell() : log_odds(0.0),
             prob(0.0),
             occ_dist(0.0),
             state(-1),
             i(0),
             j(0),
             src_i(0),
             src_j(0) {}

    /// \brief set cell values
    /// \param log_odds - log odds probability
    /// \param dist - distance to nearst obstacle
    /// \param s - state of cell
    /// \param cell_prob - probability of cell being occupied
    Cell(double log_odds, double cell_prob, double dist, int s)
                                       : log_odds(log_odds),
                                         prob(cell_prob),
                                         occ_dist(dist),
                                         state(s),
                                         i(0),
                                         j(0),
                                         src_i(0),
                                         src_j(0) {}

  };


  /// \brief Comparator for ESDF priority queue
  struct CompareDistance
  {
    bool operator()(const Cell &a, const Cell &b)
    {
      return a.occ_dist > b.occ_dist;
    }
  };


  // TODO: inherit from probability

  ///\brief Occuapncy grid mapping with known poses
  class GridMapper : public LaserScanner
  {
  public:
    /// \brief Constructs a 2D occupancy grid
    GridMapper(double resolution, double xmin, double xmax, double ymin, double ymax,
                     const LaserProperties &props, const Transform2D &Trs);

    /// \brief Compose the scan likelihood P(z|m,x)
    /// \param beam_length - range measurements from recent scan
    /// \param pose - recent pose of robot in world fram
    /// \return the probability of scan given pose and previous map
    double likelihoodFieldModel(const std::vector<float> &beam_length,
                                const Transform2D &pose) const;

    /// \brief Updates the the grid map
    /// \param beam_length - range measurements from recent scan
    /// \param pose - recent pose of robot in world fram
    void integrateScan(const std::vector<float> &beam_length,
                       const Transform2D &pose);


    /// \brief Compose a map viewable in rviz
    /// map[out] a map in row major order
    void gridMap(std::vector<int8_t> &map) const;


    void printESDF();


  private:
    /// \brief Pre-compose distance field
    void preComposeDistanceField();


    /// \brief Place cells on queue for composing ESDF
    void enqueueCell(int i, int j,
	                    int src_i, int src_j,
                      std::priority_queue<Cell, std::vector<Cell>, CompareDistance> &Q,
                      std::vector<int> &marked);

    /// \brief Compose distance field (ESDF) using fast marching method
    void euclideanSignedDistanceField();


    /// \brief updates state of cell and will call updateCellHash
    ///        to nominally update the hash table
    /// \param index - cell index in row major order in map
    /// cell[out] updated cell
    void updateCellState(Cell &cell, int index);

    /// \brief updates the hash table for the occupied and free cells
    /// \param state - occupied or free state
    ///               -1 unkown, 0 free, 1 occupied
    /// \param index - cell index in row major order in map
    void updateCellHash(int state, int index);

    /// \brief Uses Bresenham's algo to determines the
    ///        grid coordinates of the free cells
    /// \param point - end point of beam in map
    /// \param pose - robots pose in world (Twr)
    /// free_index[out] grid coordinates of the free cells
    void freeGridIndex(std::vector<int> &free_index,
                       const Vector2D &point,
                       const Transform2D &pose) const;

    /// \brief Part of Bresenham's algo for drawing lines downward
    /// \param x0 - starting x in grid
    /// \param y0 - starting y in grid
    /// \param x1 - ending x in grid
    /// \param y1 - ending y in grid
    /// free_index[out] grid coordinates of the free cells
    void lineLow(std::vector<int> &free_index,
                 int x0, int y0, int x1, int y1) const;

    /// \brief Part of Bresenham's algo for drawing lines upward
    /// \param x0 - starting x in grid
    /// \param y0 - starting y in grid
    /// \param x1 - ending x in grid
    /// \param y1 - ending y in grid
    /// free_index[out] grid coordinates of the free cells
    void lineHigh(std::vector<int> &free_index,
                  int x0, int y0, int x1, int y1) const;

    /// \brief Part of Bresenham's algo for drawing lines at 45 deg
    /// \param x0 - starting x in grid
    /// \param y0 - starting y in grid
    /// \param x1 - ending x in grid
    /// \param y1 - ending y in grid
    /// free_index[out] grid coordinates of the free cells
    void lineDiag(std::vector<int> &free_index,
                 int x0, int y0, int x1, int y1) const;

    /// \brief Converts floating point world coordinates to grid coordinates
    /// \param x - x position in world
    /// \param y - y position in world
    /// \returns the grid coordinates corresponding to a point in the world
    GridCoordinates world2Grid(double x, double y) const;

    // /// \brief Converts floating point world coordinates to grid coordinates
    // /// \param x - x position in world
    // /// \param y - y position in world
    // /// \returns the grid index in row major order
    unsigned int world2RowMajor(double x, double y) const;

    /// \brief Converts grid indices to row major oreder index
    /// \param i - row in the grid
    /// \param j - column in the grid
    /// \returns the grid index in row major order
    unsigned int grid2RowMajor(int i, int j) const;


    double prior_, prob_occ_, prob_free_;           // probabilities for cell states
    double log_odds_prior_;                         // log odds prior
    double log_odds_occ_;                           // log odds occupied
    double log_odds_free_;                          // log odds free

    double resolution_;                             // map resolution
    double max_occ_dist_;                           // max distance to obstacle
    double cell_radius_;                            // max distance to obstacle in grid dim
    double xmin_, xmax_, ymin_, ymax_;              // map dims
    int xsize_, ysize_;                             // number of discretization


    std::unordered_set<int> occ_cells_;              // occupied cell indices
    std::unordered_set<int> free_cells_;             // free cell indices

    std::vector<std::vector<double>> distances_;      // pre-compose distance to obstacles
    std::vector<Cell> map_;                         // grid map

  };

  std::ostream & operator<<(std::ostream & os, const GridCoordinates & gc);

} // end namespace
#endif
