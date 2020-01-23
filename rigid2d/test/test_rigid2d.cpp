/// \file
/// \brief unit tests for rigid2d library

#include <gtest/gtest.h>
#include <iostream>
#include <sstream>
#include <cmath>

#include "rigid2d/rigid2d.hpp"


/// \breif Test normalize Vector2D
TEST(Rigid2DTest, NormalizeVector2D)
{
  rigid2d::Vector2D v;
  v.x = 1;
  v.y = 3;

  rigid2d::NormalVec2D nv = rigid2d::normalize(v);

  ASSERT_FLOAT_EQ(nv.nx, 1.0/std::sqrt(10));
  ASSERT_FLOAT_EQ(nv.ny, 3.0/std::sqrt(10));
}


/// \brief Test vector addition
TEST(Rigid2DTest, AddVectors)
{
  rigid2d::Vector2D v1;
  v1.x = 1;
  v1.y = 2;

  rigid2d::Vector2D v2;
  v2.x = 2;
  v2.y = 3;

  rigid2d::Vector2D v = v1 + v2;


  ASSERT_EQ(v.x, 3);
  ASSERT_EQ(v.y, 5);
}


/// \brief Test vector subtraction
TEST(Rigid2DTest, SubtractVectors)
{
  rigid2d::Vector2D v1;
  v1.x = 1;
  v1.y = 2;

  rigid2d::Vector2D v2;
  v2.x = 2;
  v2.y = 3;

  rigid2d::Vector2D v = v1 - v2;


  ASSERT_EQ(v.x, -1);
  ASSERT_EQ(v.y, -1);
}


/// \brief Test vector scalar multiplication
TEST(Rigid2DTest, VectorScalarMultiplication)
{
  rigid2d::Vector2D v;
  v.x = 1;
  v.y = 2;

  double scalar = 5;

  // test from the left and right side
  rigid2d::Vector2D v_left = scalar * v;

  rigid2d::Vector2D v_right =  v * scalar;

  ASSERT_EQ(v_left.x, 5);
  ASSERT_EQ(v_left.y, 10);
  ASSERT_EQ(v_right.x, 5);
  ASSERT_EQ(v_right.y, 10);
}


/// \brief Test the length calculation of a vector
TEST(Rigid2DTest, VectorLength)
{
  rigid2d::Vector2D v;
  v.x = 1;
  v.y = 2;

  double len = rigid2d::length(v);

  ASSERT_FLOAT_EQ(len, 2.236068);
}


/// \brief Test distance between two vectors
TEST(Rigid2DTest, DistanceBetweenVectors)
{
  rigid2d::Vector2D v1;
  v1.x = 1;
  v1.y = 2;

  rigid2d::Vector2D v2;
  v2.x = 2;
  v2.y = 3;

  double dist = rigid2d::distance(v1, v2);

  ASSERT_FLOAT_EQ(dist, 1.4142135);
}


/// \brief Test angle between two vector
TEST(Rigid2DTest, AngleBetweenVectors)
{
  rigid2d::Vector2D v1;
  v1.x = 1;
  v1.y = 2;

  rigid2d::Vector2D v2;
  v2.x = 2;
  v2.y = 3;

  //radians
  double angle = rigid2d::angle(v1, v2);

  ASSERT_FLOAT_EQ(angle, 0.124355);

}




/// \brief Test transforming a Vector2D to new frame
TEST(Rigid2DTest, TransformVector2DFrame)
{
  // define vector and frame to transform
  // this is in frame c
  rigid2d::Vector2D vc;
  vc.x = 3;
  vc.y = 3;

  // define transform Tbc
  double angle = 45;
  rigid2d::Vector2D v;
  v.x = 1;
  v.y = 2;
  rigid2d::Transform2D Tbc(v, angle);

  // transform vc into frame b
  rigid2d::Vector2D vb = Tbc(vc);

  ASSERT_NEAR(vb.x, 0.023255, 1e-6);
  ASSERT_NEAR(vb.y, 6.128677, 1e-6);
}


/// \brief Test inverting a Transform2D
TEST(Rigid2DTest, InvertTransform2D)
{

  // declare desire output
  std::string output = "theta (degrees): -90 x: -1 y: 1\n";
  std::stringstream ss_out;

  // define transfor2D in invert
  double angle = rigid2d::PI/2.0; // radians
  rigid2d::Vector2D v;
  v.x = 1;
  v.y = 1;
  rigid2d::Transform2D T(v, angle);

  // inverse
  rigid2d::Transform2D Tinv = T.inv();

  // read inverse
  ss_out << Tinv;

  ASSERT_EQ(ss_out.str(), output);
}


/// \brief Test transforming a Twist2D to new frame
TEST(Rigid2DTest, TransformTwistFrame)
{
  // define twist in c
  rigid2d::Twist2D tc;
  tc.w = 1;
  tc.vx = 2;
  tc.vy = 2;

  // define transfor2D Tac
  double angle = rigid2d::PI/2.0; // radians
  rigid2d::Vector2D v;
  v.x = -1;
  v.y = 3;
  rigid2d::Transform2D Tac(v, angle);

  // transform to frame a
  rigid2d::Twist2D ta = Tac(tc);

  ASSERT_FLOAT_EQ(ta.w, 1.0);
  ASSERT_FLOAT_EQ(ta.vx, 1.0);
  ASSERT_FLOAT_EQ(ta.vy, 3.0);
}


/// \brief Test transform multiplicaton
TEST(Rigid2DTest, TransformMultiplication)
{

  // desired output
  std::string output = "theta (degrees): 90 x: -1 y: 3\n";

  // Tab
  rigid2d::Vector2D v1;
  v1.x = 1;
  v1.y = 1;

  rigid2d::Transform2D Tab(v1, rigid2d::PI/2.0); // radians

  // Tbc
  rigid2d::Vector2D v2;
  v2.x = 2;
  v2.y = 2;
  rigid2d::Transform2D Tbc(v2, 0.0); // radians

  // multiply
  rigid2d::Transform2D Tac = Tab * Tbc;

  // read output
  std::stringstream ss_out;
  ss_out << Tac;

  ASSERT_EQ(ss_out.str(), output);
}


/// \brief Test returning the displacement of a transform
TEST(Rigid2DTest, ReturnsTransformDisplacement)
{
  // T
  rigid2d::Vector2D v;
  v.x = 1;
  v.y = 3;

  rigid2d::Transform2D T(v, rigid2d::PI/2.0); // radians

  rigid2d::TransformData2D t2d = T.displacement();

  ASSERT_FLOAT_EQ(t2d.theta, rigid2d::PI/2.0);
  ASSERT_FLOAT_EQ(t2d.x, 1.0);
  ASSERT_FLOAT_EQ(t2d.y, 3.0);
}


/// \brief Tests integrating a twist
TEST(Rigid2DTest, IntegrateTwist)
{
  rigid2d::Transform2D T;

  // declare desired input
  std::string input = "90\n -1\n 3\n'";
  std::stringstream ss_in(input);

  // read in tf
  ss_in >>  T;
  // std::cout << T;


  rigid2d::Twist2D twist;
  twist.w = 1;
  twist.vx = 1;
  twist.vy = 1;

  rigid2d::Transform2D Tnew = T.integrateTwist(twist);

  std::cout << Tnew;

  rigid2d::TransformData2D Tdata = Tnew.displacement();

  ASSERT_NEAR(rigid2d::rad2deg(Tdata.theta), 147.296, 1e-3);
  ASSERT_NEAR(Tdata.x, -2.30117, 1e-3);
  ASSERT_NEAR(Tdata.y, 3.38177, 1e-3);
}



// \breif Test >> operator overload for Vector2D input
TEST(Rigid2DTest, HandlesVector2DInput)
{
  rigid2d::Vector2D v;

  // declare desired input
  std::string input = "3 3";
  std::stringstream ss_in(input);

  // read in vector
  ss_in >> v;

  // compare input to what was read in by >>
  ASSERT_EQ(ss_in.str(), input);
}


// \breif Test << operator overload for Vector2D output
TEST(Rigid2DTest, HandlesVector2DOutput)
{
  rigid2d::Vector2D v;

  // declare input
  std::string input = "3 3";

  // define desired output
  std::string output = "[3 3]\n";

  std::stringstream ss_in(input);
  std::stringstream ss_out;

  // read in vector
  ss_in >> v;

  // output vector
  ss_out << v;

  // compare output to what was read out by <<
  ASSERT_EQ(ss_out.str(), output);
}


// \breif Test >> operator overload for Twist2D input
TEST(Rigid2DTest, HandlesTwist2DInput)
{
  rigid2d::Twist2D twist;

  // declare desired input
  std::string input = "1 2 2";
  std::stringstream ss_in(input);

  // read in twist
  ss_in >> twist;


  // compare input to what was read in by >>
  ASSERT_EQ(ss_in.str(), input);
}


// \breif Test << operator overload for Twist2D output
TEST(Rigid2DTest, HandlesTwist2DOutput)
{
  rigid2d::Twist2D twist;

  // declare desired input
  std::string input = "1 2 2";

  // define desired output
  std::string output = "[1 2 2]\n";

  std::stringstream ss_in(input);
  std::stringstream ss_out;

  // read in twist
  ss_in >> twist;

  // read out twist
  ss_out << twist;

  // compare input to what was read out by <<
  ASSERT_EQ(ss_out.str(), output);
}



/// \brief Test >> operator overload for tf input
TEST(Rigid2DTest, HandlesTransformInput)
{
  rigid2d::Transform2D T;

  // declare desired input
  std::string input = "90 1 1";
  std::stringstream ss_in(input);

  // read in tf
  ss_in >> T;

  // compare input to what was read in by >>
  ASSERT_EQ(ss_in.str(), input);
}


/// \brief Test << operator overload for tf output
TEST(Rigid2DTest, HandlesTransformOutput)
{
  rigid2d::Transform2D T;

  // must give input to test output
  std::string input = "90 1 1";

  // define desired output
  std::string output = "theta (degrees): 90 x: 1 y: 1\n";

  // define in and out stringstreams
  std::stringstream ss_in(input);
  std::stringstream ss_out;

  // read the inputs
  ss_in >> T;

  // read back out
  ss_out << T;

  // compare input to what was read out by <<
  ASSERT_EQ(ss_out.str(), output);
}
















// end file
