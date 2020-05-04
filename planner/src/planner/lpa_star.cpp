// \file
/// \brief Lifelong planning A*

#include <iostream>
#include <algorithm>
#include <cmath>
#include "planner/lpa_star.hpp"


namespace planner
{

LPAstar::LPAstar(GridMap &gridmap) : gridmap(gridmap)
{
  gridmap.getGrid(grid);
  occu_cost = 1000.0;
  free_cost = 1.0;

  start_id = goal_id = curr_id = 0;

  // int i = 0;
  // for (const auto &cell : grid)
  // {
  //   std::cout << "ID: " << grid.at(i).id << std::endl;
  //   i++;
  // }


  // Cell n1;
  // n1.id = 1;
  // n1.k1 = 0.1;
  // n1.k2 = 2.0;
  //
  // Cell n2;
  // n2.id = 2;
  // n2.k1 = 1.0;
  // n2.k2 = 1.0;
  //
  // // Cell n3;
  // // n3.id = 3;
  // // n3.k1 = 1.5;
  // // n3.k2 = 50.0;
  //
  //
  // open_list.push_back(n1);
  // open_list.push_back(n2);
  // // open_list.push_back(n3);
  //
  // std::sort(open_list.begin(), open_list.end(), SortKey());
  // std::cout << "min ID: " << open_list.begin()->id << std::endl;
  //
  //
  // // auto it = std::find_if(open_list.begin(), open_list.end(), MatchesID(n3.id));
  // // std::cout << "ID: " << it->id << std::endl;

}


bool LPAstar::planPath()
{
  int i = 0;
  while(planning())
  {
    // Cell with min key
    auto it = open_list.begin();
    const Cell min_cell = (*it);
    std::cout << "Current Cell ID: " << min_cell.id << std::endl;

    // remove min cell from open list
    open_list.erase(it);


    // update the true cost and
    // examine neighbors
    if (min_cell.g > min_cell.rhs)
    {
      grid.at(min_cell.id).g = grid.at(min_cell.id).rhs;
    }

    else
    {
      grid.at(min_cell.id).g = 1e12;
    }

    std::vector<int> succ;
    neighbors(min_cell, succ);

    for(const auto &id : succ)
    {
      updateCell(id);
    }


    if (i == 100)
    {
      std::cout << "MAX iter" << std::endl;
      break;
    }
  }




  return false;
}



void LPAstar::initPath(const Vector2D &start, const Vector2D &goal)
{
  // start cell
  auto gc = gridmap.world2Grid(start.x, start.y);
  start_id = gridmap.grid2RowMajor(gc.i, gc.j);

  // add start to open list
  grid.at(start_id).rhs = 0.0;
  grid.at(start_id).h = heuristic(start_id);
  grid.at(start_id).calculateKeys();

  // std::cout << grid.at(start_id).k1 << " " << grid.at(start_id).k2 << std::endl;
  open_list.push_back(grid.at(start_id));

  // goal cell
  gc = gridmap.world2Grid(goal.x, goal.y);
  goal_id = gridmap.grid2RowMajor(gc.i, gc.j);
  grid.at(goal_id).calculateKeys();
  // std::cout << grid.at(goal_id).k1 << " " << grid.at(goal_id).k2 << std::endl;


  std::cout << start_id << " " << goal_id << std::endl;

}



void LPAstar::updateCell(int id)
{
  // not the start
  if (id != start_id)
  {
    std::vector<int> pred;
    neighbors(grid.at(id), pred);

    // pairs of predecessor ID and the cost: g(s') + c(s', u)
    std::vector<std::pair<int, double>> id_cost;

    // rhs(u)
    for(const auto &pid : pred)
    {
      const Cell cell = grid.at(pid);

      // std::cout << "succ ID: " << id << " pred ID: " << pid << std::endl;

      // cost from (s' to u)
      // occupied cell
      if (cell.state == 1 or cell.state == 2)
      {
        const auto cost = cell.g + occu_cost;
        id_cost.push_back({pid, cost});
      }

      // unoccupied or unknow cell state
      else
      {
        const auto cost = cell.g + free_cost;
        id_cost.push_back({pid, cost});
      }
    } // end loop

    // sort based on cost
    std::sort(id_cost.begin(), id_cost.end(), SortCost());

    // update rhs(u)
    const auto min_id = id_cost.front().first;
    const Cell cell = grid.at(min_id);

    // std::cout << "min ID: " << min_id << std::endl;


    // occupied cell
    if (cell.state == 1 or cell.state == 2)
    {
      grid.at(id).rhs = cell.g + occu_cost;
    }

    // unoccupied or unknow cell state
    else
    {
      grid.at(id).rhs = cell.g + free_cost;
    }
  } // end if


  // on open set then remove it
  auto it = std::find_if(open_list.begin(), open_list.end(), MatchesID(id));
  if (it != open_list.end())
  {
    open_list.erase(it);
  }

  if(!grid.at(id).locallyConsistent())
  {
    grid.at(id).calculateKeys();
    open_list.push_back(grid.at(id));
  }
}



bool LPAstar::planning()
{
  // TODO: figure out which or both keys to compare

  std::sort(open_list.begin(), open_list.end(), SortKey());
  const auto min_key1 = open_list.begin()->k1;
  const auto min_key2 = open_list.begin()->k2;

  const auto goal = grid.at(goal_id);

  if (((min_key1 < goal.k1) and (min_key2 < goal.k2)) or goal.rhs != goal.g)
  {
    return true;
  }

  std::cout << "Goal Reached" << std::endl;
  return false;
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
    // succ grid coordinates
    auto ip = i + action.at(0);
    auto jp = j + action.at(1);

    // within bounds
    if (gridmap.worldBounds(ip, jp))
    {
      id_vec.push_back(gridmap.grid2RowMajor(ip, jp));
    }
  }
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
