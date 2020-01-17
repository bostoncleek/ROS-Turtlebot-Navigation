/// \file
/// \brief driver file for rigid2d

#include "rigid2d/rigid2d.hpp"
#include <iostream>

#include <sstream>



int main()
{


  // rigid2d::Transform2D T;
  //
  // // declare desired input
  // std::string input = "90\n 1\n 1\n'";
  // std::stringstream ss_in(input);
  //
  // // read in tf
  // ss_in >>  T;
  // std::cout << T;
  //
  //
  // rigid2d::Twist2D twist;
  // twist.w = 1;
  // twist.vx = 2;
  // twist.vy = 3;
  //
  // rigid2d::Transform2D Tnew = T.integrateTwist(twist);
  //
  // std::cout << Tnew;



  // ss_in >> input;
  // std::cout << T;

  // // define transform Tbc
  // double angle = 90;
  // rigid2d::Vector2D v;
  // v.x = 1;
  // v.y = 1;
  // rigid2d::Transform2D T(v, angle);
  //
  // // transform vc into frame b
  // rigid2d::Transform2D Tinv = T.inv();

  // printf("%f\n", vb.x);
  // printf("%f\n", vb.y);



  /////////////////////////////////////////////////////////////////////




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
