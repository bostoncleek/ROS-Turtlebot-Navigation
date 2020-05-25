#ifndef DSTAR_LIGHT_GUARD_HPP
#define DSTAR_LIGHT_GUARD_HPP
// \file
/// \brief D* light version 1

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

      else if (rigid2d::almost_equal(a.k1, b.k1))
      {
        if (a.k2 < b.k2)
        {
          return true;
        }

        else
        {
          return false;
        }
      }

      // a.k1 > b.k1
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
 class DStarLight
 {
 public:
   /// \brief Construct a global path planner
   /// \param gridmap - a 2D grid
   /// \param vizd - number of visible cells for map update
   DStarLight(GridMap &gridmap, double vizd);

   /// \brief Plans a path from start to goal on the grid
   /// \return true if path is found
   void planPath();

   /// \brief simulates robot driving a path
   // from start to goal on the grid
   /// \return true goal is reached
   void pathTraversal();

   /// \brief initialize start and goal
   /// \param start - start configuration
   /// \param goal - goal configuration
   void initPath(const Vector2D &start, const Vector2D &goal);

   /// \brief Retreive the shortest path
   /// path[out] - path
   void getPath(std::vector<Vector2D> &traj) const;

   /// \brief Retreive the cells visited, usefull for debugging
   /// cells[out] - cells visited during shortes path search
   void getVisited(std::vector<Vector2D> &cells) const;

   /// \brief Compose a map viewable in rviz
   /// map[out] a map in row major order
   void getGridViz(std::vector<int8_t> &map) const;

 private:
   /// \brief Updates the cost of a cell a may remove it from the
   ///        open set if it is on it
   /// \param id - the cell ID
   void updateCell(int id);

   /// \brief Determines whether or not the goal has been reached
   /// \return true if to keep planning
   bool ifPlanning();

   /// \brief Simulates a laser scan update by updating the
   ///        cells of grid using ref_grid
   void simulateGridUpdate(std::vector<int> &cell_id);

   /// \brief Compose the neighbors of a cell
   /// \param cell - cell to examine
   /// pred[out] - IDs of all neighbors to the cell
   void neighbors(const Cell &cell, std::vector<int> &id_vec) const;

   /// \brief Compse the ID of the neighbor with the min cost
   /// \param id - starting cell ID
   /// \param exc_obs - true excludes obstacle cells
   /// \return ID of min neighbor
   int minNeighbor(int id, bool exc_obs) const;

   /// \brief Compose the heuristic cost of a cell
   ///        from id to start
   /// \param id - the cell ID
   /// \return - heuristic cost of cell
   double heuristic(int id) const;

   /// \bief Compose the traversal cost to go from
   ///       cell (id1) to cell (id2)
   /// \param id1 - ID of first cell
   /// \param id2 - ID of second cell
   /// \return - cost to go from id1 -> id2
   double edgeCost(int id1, int id2) const;

   // the grid mapper used to simulate a map
   // created by a laser scan
   GridMap gridmap;

   // complete grid with all obstalces
   // this simulates the sensor measurements
   // of map, this is the map created using
   // the
   std::vector<Cell> ref_grid;

   // The internal representation of the grid
   // by the planner
   std::vector<Cell> grid;


   std::vector<Cell> open_list;              // open list, nodes currently being considered
   double occu_cost;                         // cost of a cell being occupied
   int start_id, goal_id, curr_id;           // ID of start/goal/min cell in roadmap
   int vizd;                                 // number of cells visible from robot
   std::vector<int> visited;                 // ID of visited cells by the planner
   std::vector<Vector2D> path;               // (x/y) locations of cells in the path traversed
   bool goal_reached;                        // goal reached
 };
} // end namespace

#endif
