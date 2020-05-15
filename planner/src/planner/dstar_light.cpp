// \file
/// \brief D* light version 1

#include <iostream>
#include <algorithm>
#include <cmath>
#include <set>
#include "planner/dstar_light.hpp"


namespace planner
{

DStarLight::DStarLight(GridMap &gridmap, double vizd)
                   : gridmap(gridmap),
                     vizd(vizd)
{
  // ref grid with all the cell states
  gridmap.getGrid(ref_grid);

  // maintain internal representation of grid
  gridmap.getGrid(grid);

  // assume all cells are unoccupied
  for(auto &cell : grid)
  {
    cell.state = 0;
  }

  // costs
  occu_cost = 1000.0;
  free_cost = 1.0;

  // visibility (number of cells)
  // vizd = 5;

  start_id = goal_id = curr_id = 0;
  path.push_back(grid.at(start_id).p);
}


void DStarLight::planPath()
{
  // clear previously visited cells for viz
  visited.clear();

  while(ifPlanning())
  {
    // Cell with min key
    const auto it = open_list.begin();
    const Cell min_cell = (*it);
    // std::cout << "Current Cell ID: " << min_cell.id << std::endl;
    // std::cout << "Replanning !!!!" << std::endl;


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

      // visit predecessors of u
      std::vector<int> pred;
      neighbors(min_cell, pred);

      for(const auto &id : pred)
      {
          updateCell(id);
      }

      // update u
      updateCell(min_cell.id);
    }

  }
}


void DStarLight::pathTraversal()
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
  // collect shortest path
  path.push_back(grid.at(start_id).p);


  // update grid based on simulated sensors
  // these are the cells that are updated
  std::vector<int> cell_id;
  simulateGridUpdate(cell_id);


  // changes have taken place
  if (!cell_id.empty())
  {
    // for each cell that has been updated
    // find its neighbors and update them
    for(const auto cid : cell_id)
    {
      // all neighbors of cid
      std::vector<int> nid;
      neighbors(grid.at(cid), nid);

      for(const auto n : nid)
      {
        updateCell(n);
      }
    }

    // update cell costs/keys in open list
    for(auto &cell : open_list)
    {
      cell.h = heuristic(cell.id);
      cell.calculateKeys();
    }

    // compose shortest path
    planPath();

  }
}



void DStarLight::initPath(const Vector2D &start, const Vector2D &goal)
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
  std::cout << grid.at(goal_id).k1 << " " << grid.at(goal_id).k2 << std::endl;

}


void DStarLight::getPath(std::vector<Vector2D> &traj) const
{
  // add the path traversed so far
  traj = this->path;

  // get the remainder of the path that has not
  // been updated yet due to replanning
  auto id = start_id;
  while(id != -1)
  {
    traj.push_back(grid.at(id).p);
    id = grid.at(id).parent_id;
  }
}


void DStarLight::getVisited(std::vector<Vector2D> &cells) const
{
  for(const auto id : visited)
  {
    cells.push_back(grid.at(id).p);
  }
}


void DStarLight::getGridViz(std::vector<int8_t> &map) const
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



void DStarLight::updateCell(int id)
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
  const auto it = std::find_if(open_list.begin(), open_list.end(), MatchesID(id));
  if (it != open_list.end())
  {
    open_list.erase(it);
  }

  // not locally inconsistent then add to open list
  if(grid.at(id).rhs != grid.at(id).g)
  {
    grid.at(id).h = heuristic(id);
    grid.at(id).calculateKeys();
    open_list.push_back(grid.at(id));
  }
}



bool DStarLight::ifPlanning()
{
  // determine key for start
  grid.at(start_id).h = heuristic(start_id);
  grid.at(start_id).calculateKeys();

  // sort keys
  std::sort(open_list.begin(), open_list.end(), SortKey());

  // min keys
  const auto min_key1 = open_list.begin()->k1;
  const auto min_key2 = open_list.begin()->k2;

  const auto start = grid.at(start_id);

  // std::cout << "-------------------" << std::endl;
  // std::cout << "Min k1: " << min_key1 << std::endl;
  // std::cout << "Min k2: " << min_key2 << std::endl;
  // std::cout << "start k1: " << start.k1 << std::endl;
  // std::cout << "start k2: " << start.k2 << std::endl;
  // std::cout << "start rhs: " << start.rhs << std::endl;
  // std::cout << "start g: " << start.g << std::endl;
  // std::cout << "start h: " << start.h << std::endl;
  // std::cout << "-------------------" << std::endl;


  if (((min_key1 < start.k1) or (min_key2 < start.k2)) or (start.rhs != start.g))
  {
    return true;
  }

  // std::cout << "Shortest Path Complete" << std::endl;
  return false;
}


void DStarLight::simulateGridUpdate(std::vector<int> &cell_id)
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


  // all ID within bounding box
  for(int i = i_min; i <= i_max; i++)
  {
    for(int j = j_min; j <= j_max; j++)
    {
      const auto id = gridmap.grid2RowMajor(i, j);

      // states are not equal between the planners grid and
      // the ref_grid then assume this is an adge cost change and update
      // if (grid.at(id).state != ref_grid.at(id).state)
      if (!grid.at(id).updated)
      {
        grid.at(id).updated = true;
        grid.at(id).state = ref_grid.at(id).state;
        cell_id.push_back(id);
        // std::cout << "Updating: " << id << std::endl;
      }
    } // end inner loop
  } // end outer loop
}


void DStarLight::neighbors(const Cell &cell, std::vector<int> &id_vec) const
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


int DStarLight::minNeighbor(int id) const
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


double DStarLight::heuristic(int id) const
{
  // const auto start = grid.at(goal_id);
  const auto start = grid.at(start_id);
  const auto cell = grid.at(id);

  const auto dx = std::abs(cell.i - start.i);
  const auto dy = std::abs(cell.j - start.j);

  return std::sqrt(dx*dx + dy*dy);
}
} // end namespace
