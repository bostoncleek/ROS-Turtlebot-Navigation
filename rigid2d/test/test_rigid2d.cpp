/// \file
/// \brief unit tests for rigid2d library

#include <gtest/gtest.h>
#include <ros/ros.h>
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

  ASSERT_FLOAT_EQ(nv.nx, 1 / std::sqrt(10));
  ASSERT_FLOAT_EQ(nv.ny, 3 /std::sqrt(10));
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






int main(int argc, char * argv[])
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_rigid2d_node");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}














// end file
