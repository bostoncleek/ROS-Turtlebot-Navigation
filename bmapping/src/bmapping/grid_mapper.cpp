// \file
/// \brief A motion model for a differential drive robot

#include <iostream>
#include <stdexcept>
#include <functional>
#include <iterator>
#include <algorithm>

// #include <robot_models/probability.hpp>
#include "bmapping/grid_mapper.hpp"


namespace bmapping
{


double pdfNormal(double a, double b)
{
  if (rigid2d::almost_equal(b, 0.0))
  {
    throw std::invalid_argument("Variance in pdfNormal is 0");
  }

  const auto sqrt_inv = 1.0 / std::sqrt(2.0 * rigid2d::PI * b);
  const auto var = -0.5 * (a * a) / b;
  return sqrt_inv * std::exp(var);
}


unsigned int mapSize(double lower, double upper, double resolution)
{
  return static_cast<unsigned int> (std::ceil((upper - lower) / resolution));
}



GridMapper::GridMapper(double resolution, double xmin, double xmax,
                                          double ymin, double ymax,
                        const LaserProperties &props, const Transform2D &Trs)
    : LaserScanner(props, Trs),
      prior_(0.5),
      prob_occ_(0.90),
      prob_free_(0.35),
      log_odds_prior_(prob2LogOdds(prior_)),
      log_odds_occ_(prob2LogOdds(prob_occ_)),
      log_odds_free_(prob2LogOdds(prob_free_)),
      resolution_(resolution),
      max_occ_dist_(10.0),
      cell_radius_(mapSize(0.0, max_occ_dist_, resolution_)),
      xmin_(xmin),
      xmax_(xmax),
      ymin_(ymin),
      ymax_(ymax),
      xsize_(mapSize(xmin_, xmax_, resolution_)),
      ysize_(mapSize(ymin_, ymax_, resolution_)),
      distances_(cell_radius_, std::vector<double>(cell_radius_)),
      map_(xsize_ * ysize_, {log_odds_prior_, prior_, max_occ_dist_, -1}) // set occupies distance to max
{
  // look up table for scan likelihood
  preComposeDistanceField();
  // std::cout << "max_occ_dist_: " << max_occ_dist_ << std::endl;
  // std::cout << "cell_radius_: " << cell_radius_ << std::endl;
}




double GridMapper::likelihoodFieldModel(const std::vector<float> &beam_length,
                                        const Transform2D &pose) const
{

  // Ignores beams at max range
  // Assume all beam end points are on the map


  const auto var_hit = sigma_hit_ * sigma_hit_;
  // const auto z_rand_mult = 1.0 / range_max_;



  // End points of each beam in cartesian coordinates
  // in the robots frame relative to the map frame.
  // Ignore range if at max beam length, these are filtered
  // out in laserEndPoints
  std::vector<Vector2D> end_points;
  laserEndPoints(end_points, beam_length, pose);


  // init likelihood
  auto p = 1.0;

  // check if map has obstacles
  if (occ_cells_.size() == 0)
  {
    // std::cout<< "occ_cells_.size() == 0"  << std::endl;
    return p;
  }

  // iterate over beam end points
  for(const auto &point: end_points)
  {

    auto pz = 0.0;

    // cell the current beam end point falls into
    const auto idx = world2RowMajor(point.x, point.y);

    // the distances have been pre determined
    const auto z = map_.at(idx).occ_dist;

    // std::cout<< "d to nearst obstacle " << z << std::endl;

    // p *= z_hit_ * prob.probNormal(dist, sigma_hit_) + (z_rand_ / z_max_);
    // pz = prob.probNormal(z, var_hit);
    // pz = prob.pdfNormal(z, sigma_hit_);

    // Gaussian model
    pz += z_hit_ * pdfNormal(z, var_hit);
    // Random measurements
    pz += z_rand_ / z_max_;
    // pz += z_rand_ *z_rand_mult;
    // pz += prob.pdfNormal(z, var_hit) + prob.sampleNormal(0, 1e-5);



    p *= pz;
  }

  // p = std::clamp(p, 0.0, 1.0);

  return p;
}






void GridMapper::integrateScan(const std::vector<float> &beam_length,
                               const Transform2D &pose)
{
  std::vector<Vector2D> end_points;
  // End points of each beam in cartesian coordinates
  // in the robots frame relative to the map frame
  laserEndPoints(end_points, beam_length, pose);


  // row major order index of the map to update
  auto idx = 0;

  // Find empty indices for each laser beam
  for(unsigned int i = 0; i < end_points.size(); i++)
  {

    std::vector<int> free_index;
    freeGridIndex(free_index, end_points.at(i), pose);


    // update prob of a cell being free
    for(unsigned int j = 0; j < free_index.size(); j++)
    {
      idx = free_index.at(j);
      map_.at(idx).log_odds +=  log_odds_free_ - log_odds_prior_;

      updateCellState(map_.at(idx), idx);


    } // end inner loop


    // update prob of a cell being occupied
    idx = world2RowMajor(end_points.at(i).x, end_points.at(i).y);
    map_.at(idx).log_odds += log_odds_occ_ - log_odds_prior_;

    updateCellState(map_.at(idx), idx);

  } // end outer loop

  // update ESDF
  euclideanSignedDistanceField();
}


void GridMapper::gridMap(std::vector<int8_t> &map) const
{
  // IMPORTANT
  // Transpose map
  // convert log odds to probability
  map.resize(map_.size(), 0);

  for(unsigned int i = 0; i < map_.size(); i++)
  {
    // transpose for rviz
    auto row = i / xsize_;
    auto col = i % xsize_;
    auto idx = col * xsize_ + row;
    // auto idx = row * xsize_ + col;


    auto prob = map_.at(i).prob;


    if (prob == prior_)
    {
      map.at(idx) = -1;
    }

    else if (prob >= prob_occ_)
    {
      map.at(idx) = 100;
    }

    else if (prob <= prob_free_)
    {
      map.at(idx) = 0;
    }

    else
    {
      map.at(idx) = prob * 100;
    }

  } // end loop

}


void GridMapper::printESDF()
{
  for(unsigned int i = 0; i < map_.size(); i++)
  {
    int row = i / xsize_;
    int col = i % xsize_;
    int idx = row * xsize_ + col;
    // int idx = col * xsize_ + row;


    // std::cout << idx << ": " << map_.at(idx).occ_dist << " | ";
    printf("%d : %f |", idx, map_.at(idx).occ_dist);
    // std::cout << "row: " << row << std::endl;
    // std::cout << "col: " << col << std::endl;


    if (col == xsize_-1)
    {
      std::cout << std::endl;
    }

  }

}




void GridMapper::preComposeDistanceField()
{
  for(unsigned int i = 0; i < distances_.size(); i++)
  {
    for(unsigned int j = 0; j < distances_.size(); j++)
    {
      distances_.at(i).at(j) = std::sqrt(i*i + j*j);
      // std::cout << distances_.at(i).at(j) << " ";
    } // end inner loop
    // std::cout << std::endl;
  } // end outer loop

}


void GridMapper::enqueueCell(int i, int j,
                              int src_i, int src_j,
                              std::priority_queue<Cell, std::vector<Cell>, CompareDistance> &Q,
                              std::vector<int> &marked)
{

  // TODO: if distance to obstacle is greater than max_occ_dist_
  //       set cell distance to max dist?

  // index of cell of interest in map
  auto idx = grid2RowMajor(i, j);

  // check if visited
  if (marked.at(idx))
  {
    return;
  }

  const auto di = std::abs(i - src_i);
  const auto dj = std::abs(j - src_j);

  // if (di >= distances_.size() or dj >= distances_.size())
  // {
  //   return;
  // }
  // const auto dist = distances_.at(di).at(dj);
  auto dist = 0.0;

  try
  {
    dist = distances_.at(di).at(dj);
  }

  catch (std::out_of_range& err)
  {
    return;
  }


  if (dist > cell_radius_)
  {
    return;
  }


  // update cell
  map_.at(idx).occ_dist = dist * resolution_;
  map_.at(idx).i = i;
  map_.at(idx).j = j;
  map_.at(idx).src_i = src_i;
  map_.at(idx).src_j = src_j;

  // add to queue
  Q.push(map_.at(idx));

  // label as marked
  marked.at(idx) = 1;
}



void GridMapper::euclideanSignedDistanceField()
{
  if (occ_cells_.empty())
  {
    return;
  }


  // record cells that have been marked
  std::vector<int> marked(xsize_ * ysize_);

  // use queue in FMM
  std::priority_queue<Cell, std::vector<Cell>, CompareDistance> Q;

  // enqueue all obstacle cells
  for(auto key: occ_cells_)
  {
    map_.at(key).occ_dist = 0.0;

    // auto i = key / xsize_;
    // auto j = key % xsize_;

    map_.at(key).i = map_.at(key).src_i = key / xsize_;
    map_.at(key).j = map_.at(key).src_j = key % xsize_;


    marked.at(key) = 1;
    Q.push(map_.at(key));
  }

  // std::cout << "here" << std::endl;
  // std::cout << "xsize_" << xsize_ << std::endl;
  // std::cout << "ysize_" << ysize_ << std::endl;


  // for(unsigned int i = 0; i < map_.size(); i++)
  // {
  //   // transpose for rviz
  //   auto row = i / xsize_;
  //   auto col = i % xsize_;
  //   auto idx = row * xsize_ + col;
  //
  //   // std::cout << "idx" << idx << std::endl;
  //
  //
  //   if (map_.at(idx).state == 1)
  //   {
  //     map_.at(idx).i = map_.at(idx).src_i = row;
  //     map_.at(idx).j = map_.at(idx).src_j = col;
  //
  //     map_.at(idx).occ_dist = 0.0;
  //     marked.at(idx) = 1;
  //
  //
  //
  //     Q.push(map_.at(idx));
  //   }
  //
  //   else
  //   {
  //     map_.at(idx).occ_dist = max_occ_dist_;
  //   }
  // }



  while(!Q.empty())
  {
    Cell current_cell = Q.top();

    if (current_cell.i > 0)
    {
      enqueueCell(current_cell.i - 1, current_cell.j,
                  current_cell.src_i, current_cell.src_j,
                  Q, marked);
    }

    if (current_cell.j > 0)
    {
      enqueueCell(current_cell.i, current_cell.j - 1,
                  current_cell.src_i, current_cell.src_j,
                  Q, marked);
    }

    if (current_cell.i < xsize_ - 1)
    {
      enqueueCell(current_cell.i + 1, current_cell.j,
                  current_cell.src_i, current_cell.src_j,
                  Q, marked);
    }

    if (current_cell.j < ysize_ - 1)
    {
      enqueueCell(current_cell.i, current_cell.j + 1,
                  current_cell.src_i, current_cell.src_j,
                  Q, marked);
    }

    Q.pop();

  }

}


void GridMapper::updateCellState(Cell &cell, int index)
{
  auto prob = logOdds2Prob(cell.log_odds);

  if (prob == prior_)
  {
    cell.state = -1;
    cell.prob = prior_;

    // may need to remove a cell from a hash table
    updateCellHash(-1, index);
  }

  else if (prob >= prob_occ_)
  {
    cell.state = 1;
    cell.prob = 1;      // 100%

    // nominally update occupied hash table
    updateCellHash(1, index);
  }

  else if (prob <= prob_free_)
  {
    cell.state = 0;
    cell.prob = 0;      // 0%

    // nominally update the free hash table
    // updateCellHash(0, index);
  }

  else
  {
    cell.state = -1;    // assume it is still unkown
    cell.prob = prob;

    // may need to remove a cell from a hash table
    updateCellHash(-1, index);
  }
}


void GridMapper::updateCellHash(int state, int index)
{
  // occupied hash
  if (state == 1)
  {
    // key not in table
    auto search_occ = occ_cells_.find(index);
    if (search_occ == occ_cells_.end())
    {
      occ_cells_.insert(index);
    }
  }

  // free hash
  // else if (state == 0)
  // {
  //   auto search_free = free_cells_.find(index);
  //   if (search_free == free_cells_.end())
  //   {
  //     free_cells_.insert(index);
  //   }
  // }

  // in unkown state
  // might need to take off the
  // occupied or free hash table
  // if previously added
  else
  {
    auto search_occ = occ_cells_.find(index);
    // auto search_free = free_cells_.find(index);


    // exists in occupied
    if (search_occ != occ_cells_.end())
    {
      occ_cells_.erase(index);
      // std::cout<< "Removed from occupied cells" << std::endl;
    }

    // exists in free
    // else if (search_free != free_cells_.end())
    // {
    //   free_cells_.erase(index);
    //   // std::cout<< "Removed from free cells" << std::endl;
    // }
  }


  // std::cout<< "occupied cells size " << occ_cells_.size() <<std::endl;
  // std::cout<< "free cells size " << free_cells_.size() <<std::endl;
  //
  // std::cout<< "-----------------" << std::endl;
  // std::cout<< "Occupied cells" << std::endl;
  //
  // std::copy(occ_cells_.begin(), occ_cells_.end(),
  //                     std::ostream_iterator<int>(std::cout, " "));
  //                     std::cout<<"\n";
  //
  // std::cout<< "Free cells" << std::endl;
  //
  // std::copy(free_cells_.begin(), free_cells_.end(),
  //                     std::ostream_iterator<int>(std::cout, " "));
  //                     std::cout<<"\n";
  //
  // std::cout<< "-----------------" << std::endl;
}


void GridMapper::freeGridIndex(std::vector<int> &free_index,
                               const Vector2D &point,
                               const Transform2D &pose) const
{

  // convert to grid coordinates [x y] -> [i j]
  TransformData2D poseData = pose.displacement();
  GridCoordinates pose_grid = world2Grid(poseData.x, poseData.y);

  GridCoordinates point_grid = world2Grid(point.x, point.y);


  // start is the robot
  auto x0 = pose_grid.i;
  auto y0 = pose_grid.j;

  // end is the beam end point
  auto x1 = point_grid.i;
  auto y1 = point_grid.j;

  // deltas
  auto dx = x1 - x0;
  auto dy = y1 - y0;

  // std::cout << "Start X: " << x0 << " Start Y: " << y0 << std::endl;
  // std::cout << "Goal X: " << x1 << " Goal Y: " << y1 << std::endl;
  // std::cout << "dx: " << dx << " dy: " << dy << std::endl;

  // Case: vertical
  if (dx == 0)
  {
    // std::cout << "Plot Line Vertically" << std::endl;

    // move down
    if (dy < 0)
    {
      for(auto y = y0; y > y1; y--)
      {
        free_index.push_back(grid2RowMajor(x0, y));
      }
    }

    // move up
    else
    {
      for(auto y = y0; y < y1; y++)
      {
        free_index.push_back(grid2RowMajor(x0, y));
      }
    }

  }


  // Case: horizontal
  else if (dy == 0)
  {
    // std::cout << "Plot Line Horizontally" << std::endl;

    // move left
    if (dx < 0)
    {
      for(auto x = x0; x > x1; x--)
      {
        free_index.push_back(grid2RowMajor(x, y0));
      }
    }

    // move right
    else
    {
      for(auto x = x0; x < x1; x++)
      {
        free_index.push_back(grid2RowMajor(x, y0));
      }
    }
  }


  // Case: Draw line down and need to move farther in x than in y.
  //       Increment x every itertation
  else if (std::abs(dy) < std::abs(dx))
  {
    // std::cout << "Plot Line Low" << std::endl;
    // Octant 5
    if (x0 > x1)
    {
      // std::cout << "dx < 0" << std::endl;

      // add starting cell
      free_index.push_back(grid2RowMajor(x0, y0));

      // find middle cells
      lineLow(free_index, x1, y1, x0, y0);
    }

    // Octant 3
    else
    {
      // std::cout << "dx > 0" << std::endl;

      // add starting cell
      free_index.push_back(grid2RowMajor(x0, y0));

      // find middle cells
      lineLow(free_index, x0, y0, x1, y1);
    }
  } // end if


  // Case: Draw line down and need to move farther in y than in x.
  //       Increment y every itertation
  else if (std::abs(dy) > std::abs(dx))
  {
    // std::cout << "Plot Line High" << std::endl;

    // Octant 7
    if (y0 > y1)
    {
      // std::cout << "dy < 0" << std::endl;

      // // add starting cell
      free_index.push_back(grid2RowMajor(x0, y0));

      // find middle cells
      lineHigh(free_index, x1, y1, x0, y0);
    }

    // Octant 1
    else
    {
      // std::cout << "dy > 0" << std::endl;

      // // add starting cell
      free_index.push_back(grid2RowMajor(x0, y0));

      // find middle cells
      lineHigh(free_index, x0, y0, x1, y1);
    }
  } // end else


  // Case: Diagnol at +/- 45 degrees
  else if (std::abs(dy) == std::abs(dx))
  {
    // std::cout << "Diagnol at 45 +/- degrees" << std::endl;
    lineDiag(free_index, x0, y0, x1, y1);
  }


  else
  {
    throw std::invalid_argument("Bresenham's Line Algorithm");
  }

}


void GridMapper::lineLow(std::vector<int> &free_index,
                            int x0, int y0, int x1, int y1) const
{
  auto dx = x1 - x0;
  auto dy = y1 - y0;
  auto yi = 1;

  if (dy < 0)
  {
    yi = -1;
    dy = -dy;
  }

  auto D = 2*dy - dx;
  auto y = y0;

  // counter
  auto ctr = 0;

  for(auto x = x0; x < x1; x++)
  {
    // Start as been added already in function call
    // prevents adding the end point in the case dx < 0
    if (ctr != 0)
    {
      free_index.push_back(grid2RowMajor(x, y));
    }

    if ( D > 0)
    {
      y += yi;
      D -= 2*dx;
    }

    D += 2*dy;
    ctr++;
  } // end loop
}


void GridMapper::lineHigh(std::vector<int> &free_index,
                             int x0, int y0, int x1, int y1) const
{
  auto dx = x1 - x0;
  auto dy = y1 - y0;
  auto xi = 1;

  if (dx < 0)
  {
    xi = -1;
    dx = -dx;
  }

  auto D = 2*dx - dy;
  auto x = x0;

  // counter
  auto ctr = 0;

  for(auto y = y0; y < y1; y++)
  {
    // Start as been added already in function call
    // prevents adding the end point in the case dy < 0
    if (ctr != 0)
    {
      free_index.push_back(grid2RowMajor(x, y));
    }

    if (D > 0)
    {
      x += xi;
      D -= 2*dy;
    }

    D += 2*dx;
    ctr++;
  } // end loop
}


void GridMapper::lineDiag(std::vector<int> &free_index,
                             int x0, int y0, int x1, int y1) const
{
  const auto dx = x1 - x0;
  const auto dy = y1 - y0;

  const auto xi = (dx < 0) ? -1 : 1;
  const auto yi = (dy < 0) ? -1 : 1;

  auto x = x0;
  auto y = y0;

  while (x != x1 and y != y1)
  {
    free_index.push_back(grid2RowMajor(x, y));

    x += xi;
    y += yi;
  }

}


GridCoordinates GridMapper::world2Grid(double x, double y) const
{
  GridCoordinates grid;

  // std::cout << "X size: " << xsize_
  //           << " Y size: " << ysize_ << "\n";

  if (!(x >= xmin_ and x <= xmax_))
  {
    throw std::invalid_argument("X position Not in the bounds of the world");
  }

  if (!(y >= ymin_ and y <= ymax_))
  {
    throw std::invalid_argument("Y position Not in the bounds of the world");
  }

  // round down to the nearest cell
  // do not want to over estimate the index
  grid.i = std::floor((x - xmin_) / resolution_);
  // Case: x query == 0 and xmin < 0
  //        (0 - (-3)) / 1 = 3 == xsize
  //     or x query == xsize and xmin == 0
  //        (3 - 0) / 1 = 3 == xsize
  if (grid.i == xsize_)
  {
    grid.i--;
  }


  grid.j = std::floor((y - ymin_) / resolution_);
  // Case: y query == 0 and ymin < 0
  //     or y query == ysize and ymin == 0
  if (grid.j == ysize_)
  {
    grid.j--;
  }

  return grid;
}


unsigned int GridMapper::world2RowMajor(double x, double y) const
{
  if (!(x >= xmin_ and x <= xmax_))
  {
    throw std::invalid_argument("X position NOT in the bounds of the world");
  }

  if (!(y >= ymin_ and y <= ymax_))
  {
    throw std::invalid_argument("Y position NOT in the bounds of the world");
  }

  // round down to the nearest cell
  // do not want to over estimate the index
  auto i = std::floor((x - xmin_) / resolution_);
  // Case: x query == 0 and xmin < 0
  //        (0 - (-3)) / 1 = 3 == xsize
  //     or x query == xsize and xmin == 0
  //        (3 - 0) / 1 = 3 == xsize
  if (i == xsize_)
  {
    i--;
  }


  auto j = std::floor((y - ymin_) / resolution_);
  // Case: y query == 0 and ymin < 0
  //     or y query == ysize and ymin == 0
  if (j == ysize_)
  {
    j--;
  }

  auto index = grid2RowMajor(i, j);
  return index;
}


unsigned int GridMapper::grid2RowMajor(int i, int j) const
{
  // std::cout << "[" << i << " " << j <<"]" << std::endl;

  // IMPORTANT: swap i and j because rviz uses rows for y-axis
  //            and col for x-axis
  return i * xsize_ + j;
  // return j * xsize_ + i;  // rviz wants this
}


std::ostream & operator<<(std::ostream & os, const GridCoordinates & gc)
{
  os << "Grid Coordinates: " <<  "[" << gc.i << " " << gc.j << "]" << "\n";
  return os;
}

} // end namespace
