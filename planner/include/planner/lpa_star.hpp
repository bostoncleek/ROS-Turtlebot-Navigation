#ifndef LPA_STAR_GUARD_HPP
#define LPA_STAR_GUARD_HPP
// \file
/// \brief Lifelong planning A*

#include <iosfwd>
#include <utility>
#include <vector>

#include <rigid2d/rigid2d.hpp>
// #include <rigid2d/utilities.hpp>
// #include "planner/planner_utilities.hpp"
#include "planner/grid_map.hpp"


namespace planner
{
  using rigid2d::Vector2D;


  /// \brief Sort the cells based on k1 and the on k2
  struct SortKey
  {
    /// \brief Sorts cells
    /// \param a - cell to compare
    /// \param b - cell to compare
    /// \return true if cell a has smaller k1, if k1 == k2 then
    ///          returns true if cell a has a smaller k2
    bool operator()(const Cell &a, const Cell &b) const
    {
      if (a.k1 < b.k1)
      {
        return true;
      }

      else if (a.k1 > b.k1)
      {
        return false;
      }

      else if (a.k2 < b.k2)
      {
        return true;
      }

      else
      {
        return false;
      }
    }
  };


  /// \brief - Sort std::pait based on second element, usefull for
  ///          sorting nodes based on cost and having access to their IDs 
  struct SortCost
  {
    /// \brief Sorts cells
    /// \param a - pair to compare
    /// \param b - pair to compare
    /// \return true if pair a has smaller first element
    bool operator()(const std::pair<int, double> &a, const std::pair<int, double>&b) const
    {
      return a.second < b.second;
    }
  };



  /// \brief Finds a cell in the open list based on its ID
  struct MatchesID
  {
    int id;       // cell ID

    /// param id - the ID of a cell
    MatchesID(int id): id(id) {}

    /// \param a - cell to look for
    bool operator()(const Cell &a) const
    {
        return a.id == id;
    }
  };


  /// brief Lifelong planning A* shortest path planner
 class LPAstar
 {
 public:
   /// \brief Construct a global path planner
   /// \param gridmap - a 2D grid
   LPAstar(GridMap &gridmap);

   /// \brief Plans a path from start to goal on the grid
   /// \param start - start configuration
   /// \param goal - goal configuration
   /// \return true if path is found
   bool planPath();

   /// \brief initialize start and goal
   /// \param start - start configuration
   /// \param goal - goal configuration
   void initPath(const Vector2D &start, const Vector2D &goal);

 private:

   /// \brief Updates the cost of a cell a may remove it from the
   ///        open set if it is on it
   /// \param id - the cell ID
   void updateCell(int id);

   /// \brief Determines whether or not the goal has been reached
   /// \return true if to keep planning
   bool planning();

   /// \brief Compose the neighbors of a cell
   /// \parma cell - cell to examine
   /// pred[out] - IDs of all neighbors to the cell
   void neighbors(const Cell &cell, std::vector<int> &id_vec) const;

   /// \brief Compose the heuristic cost of a cell
   /// \param id - the cell ID
   /// \return - heuristic cost of cell
   double heuristic(int id) const;


   GridMap gridmap;                          // grid map
   std::vector<Cell> grid;                   // 2D grid of Cspace
   std::vector<Cell> open_list;              // open list, nodes currently being considered

   double occu_cost, free_cost;              // cost of a cell being occupied or free
   int start_id, goal_id, curr_id;           // ID of start/goal/current node in roadmap



 };














} // end namespace

#endif
