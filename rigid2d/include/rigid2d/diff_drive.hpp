#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for two-dimensional rigid body for a diff drive robot.

#include <cmath>
#include <iosfwd> // contains forward definitions for iostream objects

#include "rigid2d/rigid2d.hpp"


namespace rigid2d
{

  /// \brief pose for a 2D diff drive robot
  struct Pose
  {
    double theta = 0.0;
    double x = 0.0;
    double y = 0.0;
  };

  /// \brief returns the angular velocity of each wheel
  struct WheelVelocities
  {
    double ul = 0.0;
    double ur = 0.0;
  };


  class DiffDrive
  {
  public:
      /// \brief the default constructor creates a robot at (0,0,0), with a fixed wheel base and wheel radius
      DiffDrive();


      /// \brief create a DiffDrive model by specifying the pose, and geometry
      ///
      /// \param pose - the current position of the robot
      /// \param wheel_base - the distance between the wheel centers
      /// \param wheel_radius - the raidus of the wheels
      DiffDrive(const Pose &pose, double wheel_base, double wheel_radius);


      /// \brief determine the wheel velocities required to make the robot
      ///        move with the desired linear and angular velocities
      /// \param twist - the desired twist in the body frame of the robot
      /// \returns - the wheel velocities to use
      /// \throws std::exception
      WheelVelocities twistToWheels(const Twist2D &twist);

      /// \brief determine the body twist of the robot from its wheel velocities
      /// \param vel - the velocities of the wheels, assumed to be held constant
      ///  for one time unit
      /// \returns twist in the original body frame of the
      Twist2D wheelsToTwist(const WheelVelocities &vel);


      /// \brief Update the robot's odometry based on the current encoder readings
      /// \param left - the left encoder angle (in radians)
      /// \param right - the right encoder angle (in radians)
      /// \return the velocities of each wheel, assuming that they have been
      /// constant since the last call to updateOdometry
      WheelVelocities updateOdometry(double left, double right);


      /// \brief update the odometry of the diff drive robot, assuming that
      /// it follows the given body twist for one time  unit
      /// \param cmd - the twist command to send to the robot
      // TODO: update wheel encoder angles
      void feedforward(const Twist2D &cmd);


      /// \brief get the current pose of the robot
      Pose pose();

      /// \brief get the wheel speeds, based on the last encoder update
      WheelVelocities wheelVelocities() const;

      /// \brief reset the robot to the given position/orientation
      void reset(Pose ps);


    private:
      double theta, x, y, wheel_base, wheel_radius; // theta, x, y of pose and geometry
      double left_curr, right_curr; // left and right current encoder readings
      double ul, ur; // the current wheel velocities
    };
}



#endif
