// \file
/// \brief Lifelong planning A*

#include <iostream>
#include <algorithm>
#include <cmath>
#include <set>
#include "planner/lpa_star.hpp"


namespace planner
{

LPAstar::LPAstar(GridMap &gridmap) : gridmap(gridmap)
{
  // ref grid with all the cell states
  gridmap.getGrid(ref_grid);

  // maintain internal representation
  // of the ref grid
  // but pretend we do not know the cell states
  gridmap.getGrid(grid);

  for(auto &cell : grid)
  {
    cell.state = 0;
  }

  // costs
  occu_cost = 1000.0;
  free_cost = 1.0;

  // visibility (number of cells)
  vizd = 1;

  start_id = goal_id = curr_id = 0;
  path.push_back(grid.at(start_id).p);
}


void LPAstar::planPath()
{
  visited.clear();


  int i = 0;
  while(ifPlanning())
  {
    // Cell with min key
    auto it = open_list.begin();
    const Cell min_cell = (*it);
    // std::cout << "Current Cell ID: " << min_cell.id << std::endl;
    std::cout << "Replanning !!!!" << std::endl;


    // remove min cell from open list
    open_list.erase(it);

    curr_id = min_cell.id;
    visited.push_back(min_cell.id);

    // update the true cost and
    // examine neighbors
    if (min_cell.g > min_cell.rhs)
    {
      // update cost of u
      grid.at(min_cell.id).g = grid.at(min_cell.id).rhs;

      // visit predecessors of u
      std::vector<int> pred;
      neighbors(min_cell, pred);

      for(const auto &id : pred)
      {
        updateCell(id);
      }
    }

    else
    {
      // set cost u to large number
      grid.at(min_cell.id).g = 1e12;

      // visit successors of u
      std::vector<int> pred;
      neighbors(min_cell, pred);

      for(const auto &id : pred)
      {
        updateCell(id);
      }

      // update u
      updateCell(min_cell.id);
    }


    if (i == 100)
    {
      std::cout << "MAX iter" << std::endl;
      break;
    }
  }

  // return false;
}


void LPAstar::pathTraversal()
{
  // goal reached
  if (start_id == goal_id)
  {
    std::cout << "Goal Reached" << std::endl;
    return;
  }

  // move from start to the min successor
  start_id = minNeighbor(start_id);
  // std::cout << "Start ID: " << start_id << std::endl;
  path.push_back(grid.at(start_id).p);


  // update grid based on simulated sensors
  simulateGridUpdate();

  // compose shortest path
  planPath();
}



void LPAstar::initPath(const Vector2D &start, const Vector2D &goal)
{
  // start cell
  auto gc = gridmap.world2Grid(start.x, start.y);
  start_id = gridmap.grid2RowMajor(gc.i, gc.j);

  // goal cell
  gc = gridmap.world2Grid(goal.x, goal.y);
  goal_id = gridmap.grid2RowMajor(gc.i, gc.j);

  // costs and key for goal
  grid.at(goal_id).rhs = 0.0;
  grid.at(goal_id).h = heuristic(start_id);
  grid.at(goal_id).calculateKeys();

  // add goal to open list
  open_list.push_back(grid.at(goal_id));

  std::cout << start_id << " " << goal_id << std::endl;
}


void LPAstar::getPath(std::vector<Vector2D> &path) const
{
  // std::cout << "Retreiving Path" << std::endl;
  path = this->path;


  // auto id = curr_id;
  //
  // // std::cout << "Current ID: " << id << std::endl;
  // while(id != -1)
  // {
  //   const Cell cell = grid.at(id);
  //   path.push_back(cell.p);
  //
  //   id = cell.parent_id;
  //   // std::cout << "Parent ID: " << id << std::endl;
  // }
  //
  // std::reverse(path.begin(), path.end());
}


void LPAstar::getVisited(std::vector<Vector2D> &cells) const
{
  for(const auto id : visited)
  {
    cells.push_back(grid.at(id).p);
  }
}


void LPAstar::getGridViz(std::vector<int8_t> &map) const
{
  // IMPORTANT
  // convert from column to row
  map.resize(grid.size());

  // number of discretizations
  std::vector<int> gs = gridmap.getGridSize();
  const auto xsize = gs.at(0);
  const auto ysize = gs.at(1);

  for(unsigned int i = 0; i < grid.size(); i++)
  {
    // convert from column major order to row major order
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



void LPAstar::updateCell(int id)
{
  // not the start
  if (id != goal_id)
  {
    // successor with min cost
    const auto min_id = minNeighbor(id);

    // update rhs(u)
    // occupied cell
    if (grid.at(min_id).state == 1 or grid.at(min_id).state == 2)
    {
      grid.at(id).rhs = grid.at(min_id).g + occu_cost;
    }

    // unoccupied or unknow cell state
    else
    {
      grid.at(id).rhs = grid.at(min_id).g + free_cost;
    }

    // update parent
    grid.at(id).parent_id = min_id;
  }


  // on open set then remove it
  auto it = std::find_if(open_list.begin(), open_list.end(), MatchesID(id));
  if (it != open_list.end())
  {
    open_list.erase(it);
  }


  // std::cout << "rhs: " << grid.at(id).rhs << std::endl;
  // std::cout << "g: " << grid.at(id).g << std::endl;


  // not locally inconsistent then add to open list
  if(!grid.at(id).locallyConsistent())
  {
    grid.at(id).h = heuristic(id);
    grid.at(id).calculateKeys();
    open_list.push_back(grid.at(id));
  }
}



bool LPAstar::ifPlanning()
{
  // TODO: figure out which or both keys to compare

  // determine key for start
  grid.at(start_id).h = heuristic(start_id);
  grid.at(start_id).calculateKeys();

  // sort keys
  std::sort(open_list.begin(), open_list.end(), SortKey());

  // min keys
  const auto min_key1 = open_list.begin()->k1;
  const auto min_key2 = open_list.begin()->k2;

  const auto start = grid.at(start_id);

  std::cout << "-------------------" << std::endl;
  std::cout << "Min k1: " << min_key1 << std::endl;
  std::cout << "Min k2: " << min_key2 << std::endl;
  std::cout << "start k1: " << start.k1 << std::endl;
  std::cout << "start k2: " << start.k2 << std::endl;
  std::cout << "start rhs: " << start.rhs << std::endl;
  std::cout << "start g: " << start.g << std::endl;
  std::cout << "-------------------" << std::endl;



  if (((min_key1 < start.k1) or (min_key2 < start.k2)) or (start.rhs != start.g))
  // if (((min_key1 < start.k1) and (min_key2 < start.k2)))
  {
    return true;
  }

  std::cout << "Shortest Path Complete" << std::endl;
  return false;
}


void LPAstar::simulateGridUpdate()
{
  // TODO: check bounds on these
  // TODO: make sure to only update it the cell has not been updated already


  // index of u
  const auto iu = grid.at(start_id).i;
  const auto ju = grid.at(start_id).j;

  // bounding box coordinates around min cell (u)
  auto i_min = iu - vizd;
  auto i_max = iu + vizd;

  auto j_min = ju - vizd;
  auto j_max = ju + vizd;

  // number of discretizations
  std::vector<int> gs = gridmap.getGridSize();
  const auto xsize = gs.at(0);
  const auto ysize = gs.at(1);

  // adjust based on bounds of grid
  if (i_min < 0)
  {
    i_min = 0;
  }

  if (j_min < 0)
  {
    j_min = 0;
  }

  if (i_max >= xsize)
  {
    i_max = xsize-1;
  }

  if (j_max >= ysize)
  {
    j_max = ysize-1;
  }


  // neighbors of u
  // need this to determine which ones to update
  // create set to see if need to update a directed edge
  // not all visible cells that are updated
  // are edges from (u, v)
  std::vector<int> nid;
  neighbors(grid.at(start_id), nid);
  std::set<int> nid_set;

  for(const auto n : nid)
  {
    nid_set.insert(n);
  }


  // all ID within bounding box
  for(int i = i_min; i <= i_max; i++)
  {
    for(int j = j_min; j <= j_max; j++)
    {
      const auto id = gridmap.grid2RowMajor(i, j);

      // std::cout << "Updating: " << id << std::endl;
      if (!gridmap.worldBounds(i,j))
      {
        std::cout << "not in bounds: " << id << std::endl;
      }
      grid.at(id).state = ref_grid.at(id).state;

      // directed edge (u, v)
      if (nid_set.find(id) != nid_set.end())
      {
        // update edge cost (c) which here is the state
        // grid.at(id).state = ref_grid.at(id).state;

        // update (u), the current min node
        // std::cout << "Updating !!!!!!!!! " <<  std::endl;
        updateCell(start_id);
      }
    } // end inner loop
  } // end outer loop


  // update cells in open list
  for(auto &cell : open_list)
  {
    cell.calculateKeys();
  }
}







void LPAstar::neighbors(const Cell &cell, std::vector<int> &id_vec) const
{
  // actions
  std::vector<std::vector<int>> actions = {{0, -1}, {0, 1},
                                           {-1, 0}, {1, 0},
                                           {-1, -1}, {-1, 1},
                                           {1, -1},  {1, 1}};
  // cell grid coordinates
  const auto i = cell.i;
  const auto j = cell.j;

  for(const auto &action : actions)
  {
    // std::cout << action.at(0) << " " << action.at(1) << std::endl;
    // grid coordinates of neighbor
    const auto in = i + action.at(0);
    const auto jn = j + action.at(1);
    const auto idn = gridmap.grid2RowMajor(in, jn);

    // within bounds and not an obstacle
    if (gridmap.worldBounds(in, jn))
    // if (gridmap.worldBounds(in, jn) and (grid.at(idn).state != 1 and grid.at(idn).state != 2))
    {
      id_vec.push_back(idn);
    }
  }
}


int LPAstar::minNeighbor(int id) const
{
  std::vector<int> neigh;
  neighbors(grid.at(id), neigh);

  // pairs of successor ID and the cost: g(s') + c(s', u)
  std::vector<std::pair<int, double>> id_cost;

  for(const auto &nid : neigh)
  {
    const Cell cell = grid.at(nid);

    // std::cout << "succ ID: " << id << " pred ID: " << pid << std::endl;

    // cost from (s' to u)
    // occupied cell
    if (cell.state == 1 or cell.state == 2)
    {
      const auto cost = cell.g + occu_cost;
      id_cost.push_back({nid, cost});
    }

    // unoccupied or unknow cell state
    else
    {
      const auto cost = cell.g + free_cost;
      id_cost.push_back({nid, cost});
    }
  } // end loop

  // sort based on cost
  std::sort(id_cost.begin(), id_cost.end(), SortCost());

  return id_cost.front().first;
}


double LPAstar::heuristic(int id) const
{
  const auto goal = grid.at(goal_id);
  const auto cell = grid.at(id);

  const auto dx = std::abs(cell.i - goal.i);
  const auto dy = std::abs(cell.j - goal.j);

  return std::sqrt(dx*dx + dy*dy);
}




















} // end namespace
