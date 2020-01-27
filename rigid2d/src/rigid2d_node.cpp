/// \file
/// \brief driver file for rigid2d

#include <iostream>
#include <sstream>

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"


int main(int argc, char *argv[])
{

  // // rigid2d::Twist2D twist_in;
  // // rigid2d::Twist2D twist_out;
  // //
  // // rigid2d::WheelVelocities vel_in;
  // // rigid2d::WheelVelocities vel_out;
  // //
  // // rigid2d::Pose pose;
  // //
  // // twist_in.w = 1;
  // // twist_in.vx = 1;
  // // twist_in.vy = 0;
  // //
  // // double wheel_radius = 0.02;
  // // double wheel_base = 1.0;
  // //
  // // rigid2d::DiffDrive drive(pose, wheel_base, wheel_radius);
  // //
  // //
  // //
  // // vel_out = drive.twistToWheels(twist_in);
  // //
  // //
  // // std::cout << "wheel vel" << std::endl;
  // // std::cout << vel_out.ul << " " << vel_out.ur << std::endl;
  // // std::cout << "------------------" << std::endl;
  // //
  // // vel_in = vel_out;
  // // twist_out = drive.wheelsToTwist(vel_in);
  // //
  // // std::cout << "twist" << std::endl;
  // // std::cout << twist_out.w << " " << twist_out.vx << " " << twist_out.vy << std::endl;
  // // std::cout << "------------------" << std::endl;
  //
  //
  //
  //
  // rigid2d::Twist2D cmd;
  //
  // rigid2d::Pose pose_1;
  // rigid2d::Pose pose_2;
  //
  // rigid2d::WheelEncoders encoder_1;
  // rigid2d::WheelEncoders encoder_2;
  //
  // rigid2d::WheelVelocities vel_1;
  // rigid2d::WheelVelocities vel_2;
  //
  //
  // pose_1.theta = 0.0;
  // pose_1.x = 0.0;
  // pose_1.y = 0.0;
  //
  // pose_2 = pose_1;
  //
  // double wheel_radius = 0.02;
  // double wheel_base = 1.0;
  //
  // rigid2d::DiffDrive drive_1(pose_1, wheel_base, wheel_radius);
  // rigid2d::DiffDrive drive_2(pose_2, wheel_base, wheel_radius);
  //
  //
  // cmd.w = 0.0;
  // cmd.vx = 0.05;
  // cmd.vy = 0.0;
  //
  // // apply forward twist
  // drive_1.feedforward(cmd);
  //
  // pose_1 = drive_1.pose();
  //
  // encoder_1 = drive_1.getEncoders();
  //
  // vel_1 = drive_1.wheelVelocities();
  //
  // std::cout << "feedforward" << std::endl;
  // std::cout << "------------------" << std::endl;
  // std::cout << "pose" << std::endl;
  // std::cout << pose_1.theta << " " << pose_1.x << " " << pose_1.y << std::endl;
  // std::cout << "------------------" << std::endl;
  // std::cout << "encoders" << std::endl;
  // std::cout << encoder_1.left << " " << encoder_1.right << std::endl;
  // std::cout << "------------------" << std::endl;
  // std::cout << "wheel vel" << std::endl;
  // std::cout << vel_1.ul << " " << vel_1.ur << std::endl;
  // std::cout << "------------------" << std::endl;
  //
  //
  // vel_2 = drive_2.updateOdometry(encoder_1.left, encoder_1.right);
  //
  // pose_2 = drive_2.pose();
  //
  // encoder_2 = drive_2.getEncoders();
  //
  // vel_2 = drive_2.wheelVelocities();
  //
  //
  // std::cout << "updateOdometry" << std::endl;
  // std::cout << "------------------" << std::endl;
  // std::cout << "pose" << std::endl;
  // std::cout << pose_2.theta << " " << pose_2.x << " " << pose_2.y << std::endl;
  // std::cout << "------------------" << std::endl;
  // std::cout << "encoders" << std::endl;
  // std::cout << encoder_2.left << " " << encoder_2.right << std::endl;
  // std::cout << "------------------" << std::endl;
  // std::cout << "wheel vel" << std::endl;
  // std::cout << vel_2.ul << " " << vel_2.ur << std::endl;
  // std::cout << "------------------" << std::endl;





/////////////////////////////////////////////////////////////////////////////

  // inputs
  rigid2d::Transform2D Tab, Tbc;
  rigid2d::Vector2D v;
  rigid2d::Twist2D t;
  char frame;

  // compute these outputs
  rigid2d::Transform2D Tba, Tcb, Tac, Tca;


  std::cout << "----------------------" << std::endl;
  std::cout << "Enter T_ab" << std::endl;
  std::cin >> Tab;


  std::cout << "Enter T_bc" << std::endl;
  std::cin >> Tbc;
  std::cout << "----------------------" << std::endl;

  // compose some inverses
  Tcb = Tbc.inv();
  Tba = Tab.inv();


  std::cout << "----------------------" << std::endl;
  ////////////////////////////////
  // Tab
  std::cout << "T_ab: ";
  std::cout << Tab;
  ////////////////////////////////

  ////////////////////////////////
  // Tba
  std::cout << "T_ba: ";
  std::cout << Tba;
  ////////////////////////////////

  ////////////////////////////////
  // Tbc
  std::cout << "T_bc: ";
  std::cout << Tbc;
  ////////////////////////////////

  ////////////////////////////////
  // Tcb
  std::cout << "T_cb: ";
  std::cout << Tcb;
  ////////////////////////////////

  ////////////////////////////////
  // Tac
  Tac = Tab * Tbc;
  std::cout << "T_ac: ";
  std::cout << Tac;
  ////////////////////////////////

  ////////////////////////////////
  // Tca
  Tca = Tcb * Tba;
  std::cout << "T_ca: ";
  std::cout << Tca;
  ////////////////////////////////


  std::cout << "----------------------" << std::endl;
  std::cout << "----------------------" << std::endl;


  ////////////////////////////////
  // vector in frame
  std::cout << "Enter a vector"<< std::endl;
  std::cin >> v;

  std::cout << "Enter the frame: 'a', 'b', 'c'" << std::endl;
  std::cin >> frame;

  if (frame == 'a')
  {
    // in frame a
    std::cout << "Vector in frame a: ";
    std::cout << v;

    // in frame b
    std::cout << "Vector in frame b: ";
    // rotate
    rigid2d::Vector2D vb = Tba(v);
    std::cout << vb;

    // in frame c
    std::cout << "Vector in frame c: ";
    // rotate
    rigid2d::Vector2D vc = Tca(v);
    std::cout << vc;
  }

  else if (frame == 'b')
  {
    // in frame a
    std::cout << "Vector in frame a: ";
    // rotate
    rigid2d::Vector2D va = Tab(v);
    std::cout << va;

    // in frame b
    std::cout << "Vector in frame b: ";
    std::cout << v;

    // in frame c
    std::cout << "Vector in frame c: ";
    // rotate
    rigid2d::Vector2D vc = Tcb(v);
    std::cout << vc;
  }

  else if (frame == 'c')
  {
    // in frame a
    std::cout << "Vector in frame a: ";
    // rotate
    rigid2d::Vector2D va = Tac(v);
    std::cout << va;

    // in frame b
    std::cout << "Vector in frame b: ";
    // rotate
    rigid2d::Vector2D vb = Tbc(v);
    std::cout << vb;

    // in frame c
    std::cout << "Vector in frame c: ";
    std::cout << v;

  }


  std::cout << "----------------------" << std::endl;
  std::cout << "----------------------" << std::endl;


  ////////////////////////////////
  // twist in frame
  std::cout << "Enter a twist"<< std::endl;
  std::cin >> t;


  if (frame == 'a')
  {
    // in frame a
    std::cout << "Twist in frame a: ";
    std::cout << t;

    // in frame b
    std::cout << "Twist in frame b: ";
    rigid2d::Twist2D tb = Tba(t);
    std::cout << tb;

    // in frame c
    std::cout << "Twist in frame c: ";
    rigid2d::Twist2D tc = Tca(t);
    std::cout << tc;
  }


  else if (frame == 'b')
  {
    // in frame a
    std::cout << "Twist in frame a: ";
    rigid2d::Twist2D ta = Tab(t);
    std::cout << ta;


    // in frame b
    std::cout << "Twist in frame b: ";
    std::cout << t;


    // in frame c
    std::cout << "Twist in frame c: ";
    rigid2d::Twist2D tc = Tcb(t);
    std::cout << tc;
  }

  else if (frame == 'c')
  {
    // in frame a
    std::cout << "Twist in frame a: ";
    rigid2d::Twist2D ta = Tac(t);
    std::cout << ta;


    // in frame b
    std::cout << "Twist in frame b: ";
    rigid2d::Twist2D tb = Tbc(t);
    std::cout << tb;


    // in frame c
    std::cout << "Twist in frame c: ";
    std::cout << t;
  }
  std::cout << "----------------------" << std::endl;

  return 0;
}


// edn file
