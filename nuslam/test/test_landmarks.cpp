/// \file
/// \brief unit tests for nuslam library

#include <gtest/gtest.h>
#include <iostream>
#include <sstream>
#include <cmath>

#include "nuslam/landmarks.hpp"
#include "rigid2d/rigid2d.hpp"


TEST(CircleFitting, FitCircle1)
{
  using nuslam::LaserProperties;
  using nuslam::Landmarks;

  using rigid2d::Vector2D;
  using rigid2d::deg2rad;


  // lidar properties
  double beam_min = 0.12, beam_max = deg2rad(360.0);
  double beam_delta = deg2rad(0.0);
  double range_min = 0.12, range_max = 3.5;

  LaserProperties props(beam_min, beam_max, beam_delta, range_min, range_max);

  // landmark classifier
  double epsilon = 0.05;
  Landmarks landmarks(props, epsilon);


  Vector2D v1(1.0, 7.0);
  Vector2D v2(2.0, 6.0);
  Vector2D v3(5.0, 8.0);
  Vector2D v4(7.0, 7.0);
  Vector2D v5(9.0, 5.0);
  Vector2D v6(3.0, 7.0);

  nuslam::Cluster cluster;
  cluster.points.push_back(v1);
  cluster.points.push_back(v2);
  cluster.points.push_back(v3);
  cluster.points.push_back(v4);
  cluster.points.push_back(v5);
  cluster.points.push_back(v6);

  landmarks.centroid(cluster);
  landmarks.shiftCentroidToOrigin(cluster);
  landmarks.composeCircle(cluster);



  ASSERT_NEAR(cluster.x_hat, 4.615482, 1e-4);
  ASSERT_NEAR(cluster.y_hat, 2.807354, 1e-4);
  ASSERT_NEAR(cluster.radius, 4.8275, 1e-4);
}



TEST(CircleFitting, FitCircle2)
{
  using nuslam::LaserProperties;
  using nuslam::Landmarks;

  using rigid2d::Vector2D;
  using rigid2d::deg2rad;


  // lidar properties
  double beam_min = 0.12, beam_max = deg2rad(360.0);
  double beam_delta = deg2rad(0.0);
  double range_min = 0.12, range_max = 3.5;

  LaserProperties props(beam_min, beam_max, beam_delta, range_min, range_max);

  // landmark classifier
  double epsilon = 0.05;
  Landmarks landmarks(props, epsilon);


  Vector2D v1(-1.0, 0.0);
  Vector2D v2(-0.3, -0.06);
  Vector2D v3(0.3, 0.1);
  Vector2D v4(1.0, 0.0);

  nuslam::Cluster cluster;
  cluster.points.push_back(v1);
  cluster.points.push_back(v2);
  cluster.points.push_back(v3);
  cluster.points.push_back(v4);



  landmarks.centroid(cluster);
  landmarks.shiftCentroidToOrigin(cluster);
  landmarks.composeCircle(cluster);



  ASSERT_NEAR(cluster.x_hat, 0.4908357, 1e-4);
  ASSERT_NEAR(cluster.y_hat, -22.15212, 1e-4);
  ASSERT_NEAR(cluster.radius, 22.17979, 1e-4);
}
