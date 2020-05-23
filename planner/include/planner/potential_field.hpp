#ifndef POTENTIAL_FIELD_HPP
#define POTENTIAL_FIELD_HPP
/// \file
/// \brief Potenial fields for global planning in continous space


#include <vector>
#include <iosfwd>
#include <cmath>

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/utilities.hpp"
#include "planner/planner_utilities.hpp"


namespace planner
{
  using rigid2d::Vector2D;
  using rigid2d::euclideanDistance;

  /// \brief Compose the descent direction
  /// \param Urep - repulsive gradient
  /// \param Uatt - attractive gradient
  /// \return the descent direction (2D) for gradient descent (steepest direction)
  Vector2D descentDirection(const Vector2D &Urep, const Vector2D &Uatt);

  /// \brief Global path planning using attractive and repulsive fields
  class PotentialField
  {
  public:
    /// \brief Constructs a potential field global planner
    /// \param obs_map - coordinates of the vertices of all the obstacles
    /// \param eps - tolerance for reaching goal
    /// \param step - step size in gradient descent
    /// \param dthesh - distance when attractive potentail switches
    ///                 from conic to quadratic
    /// \param qthresh - repulsive potenial range of influence
    /// \param w_att - weight attractive potential
    /// \param w_rep - weight repulsive potential
    PotentialField(obstacle_map &obs_map,
                   double eps, double step,
                   double dthresh, double qthresh,
                   double w_att, double w_rep);


    /// \brief - Initialize path with start to goal
    /// \param start - starting position
    /// \param goal - goal position
    void initPath(const Vector2D &start, const Vector2D &goal);

    /// \brief - Construct path from start to goal
    /// \return true if goal reached
    bool planPath();

    /// \brief Coordinates of nodes in path
    /// path[out] - (x/y) locations of each node
    void getPath(std::vector<Vector2D> &path) const;

  private:
    /// \brief Compose the accumulative repulsive gradient
    ///        from closes point on each obstacle to current position
    /// \return cummulative gradient 2D (du/dx, du/dy)
    Vector2D accumulateRepulsiveGradient() const;

    /// \brief Compose repulsive gradient from a single obstacle to current position
    /// \param q0 - closes point on an obstacle
    /// \param d - euclidean distance from  current position (q) to closest point (q0)
    /// \return repulsive gradient 2D (du/dx, du/dy)
    Vector2D repulsiveGradient(const Vector2D &q0, double d) const;

    /// \brief Compose attractive gradient to goal
    /// \return attractive gradient 2D (du/dx, du/dy)
    Vector2D attractiveGradient() const;

    /// \brief Within tolerance of goal
    /// \returns true if within tolerance
    bool goalReached();

    /// \brief Performs one iteration of gradient descent using the attractive and repulsive
    ///        gradients to update the current position (q)
    /// \param Dn - descent direction
    void oneStepGD(const Vector2D &Dn);


    obstacle_map obs_map;               // coordinates of the vertices of all the obstacles
    std::vector<Vector2D> path;         // current path (x/y)
    Vector2D q, qg;                     // current position and goal position
    double eps, step;                   // tolerance for reaching goal and step size in gradient descent
    double dthresh, qthresh;            // attractive potentail switches from conic to quadratic and repulsive potenial range of influence
    double w_att, w_rep;                // weight the attractive and repulsive gradient
    bool goal_reached;                  // path complete

  };
} // end namespace


#endif
