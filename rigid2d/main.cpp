/// \file
/// \brief driver file for rigid2d


#include "rigid2d.hpp"
#include <iostream>





int main()
{

  // inputs
  rigid2d::Transform2D Tab;
  rigid2d::Transform2D Tbc;
  rigid2d::Vector2D v;
  rigid2d::Twist2D t;
  char frame;

  // compute these outputs
  rigid2d::Transform2D Tac;
  rigid2d::Transform2D Tca;


  std::cout << "----------------------" << std::endl;
  std::cout << "Enter T_ab" << std::endl;
  rigid2d::operator>>(std::cin, Tab);

  std::cout << "Enter T_bc" << std::endl;
  rigid2d::operator>>(std::cin, Tbc);
  std::cout << "----------------------" << std::endl;


  std::cout << "----------------------" << std::endl;
  std::cout << "T_ab: ";
  rigid2d::operator<<(std::cout, Tab);

  std::cout << "T_bc: ";
  rigid2d::operator<<(std::cout, Tbc);


  ////////////////////////////////
  // Tac
  Tac = rigid2d::operator*(Tab, Tbc);

  std::cout << "T_ac: ";
  rigid2d::operator<<(std::cout, Tac);
  ////////////////////////////////


  ////////////////////////////////
  // Tca
  rigid2d::Transform2D Tcb = Tbc.inv();
  rigid2d::Transform2D Tba = Tab.inv();

  Tca = rigid2d::operator*(Tcb, Tba);

  std::cout << "T_ca: ";
  rigid2d::operator<<(std::cout, Tca);
  ////////////////////////////////


  std::cout << "----------------------" << std::endl;
  std::cout << "----------------------" << std::endl;


  ////////////////////////////////
  // vector in frame
  std::cout << "Enter a vector"<< std::endl;
  rigid2d::operator>>(std::cin, v);

  std::cout << "Enter the frame: 'a', 'b', 'c'" << std::endl;
  std::cin >> frame;

  if (frame == 'a')
  {
    // in frame a
    std::cout << "Vector in frame a: ";
    rigid2d::operator<<(std::cout, v);

    // in frame b
    std::cout << "Vector in frame b: ";
    // rotate
    rigid2d::Vector2D vb = Tba.operator()(v);
    vb.x += v.x;
    vb.y += v.y;
    // translate
    rigid2d::operator<<(std::cout, vb);

    // in frame c
    std::cout << "Vector in frame c: ";
    // rotate
    rigid2d::Vector2D vc = Tca.operator()(v);
    vc.x += v.x;
    vc.y += v.y;
    // translate
    rigid2d::operator<<(std::cout, vc);
  }

  else if (frame == 'b')
  {
    // in frame a
    std::cout << "Vector in frame a: ";
    // rotate
    rigid2d::Vector2D va = Tab.operator()(v);
    va.x += v.x;
    va.y += v.y;
    // translate
    rigid2d::operator<<(std::cout, va);

    // in frame b
    std::cout << "Vector in frame b: ";
    rigid2d::operator<<(std::cout, v);

    // in frame c
    std::cout << "Vector in frame c: ";
    // rotate
    rigid2d::Vector2D vc = Tcb.operator()(v);
    vc.x += v.x;
    vc.y += v.y;
    // translate
    rigid2d::operator<<(std::cout, vc);
  }

  else if (frame == 'c')
  {
    // in frame a
    std::cout << "Vector in frame a: ";
    // rotate
    rigid2d::Vector2D va = Tac.operator()(v);
    va.x += v.x;
    va.y += v.y;
    // translate
    rigid2d::operator<<(std::cout, va);


    // in frame b
    std::cout << "Vector in frame b: ";
    // rotate
    rigid2d::Vector2D vb = Tbc.operator()(v);
    vb.x += v.x;
    vb.y += v.y;
    // translate
    rigid2d::operator<<(std::cout, vb);


    // in frame c
    std::cout << "Vector in frame c: ";
    rigid2d::operator<<(std::cout, v);
  }


  std::cout << "----------------------" << std::endl;
  std::cout << "----------------------" << std::endl;


  ////////////////////////////////
  // twist in frame
  std::cout << "Enter a twist"<< std::endl;
  rigid2d::operator>>(std::cin, t);


  if (frame == 'a')
  {
    // in frame a
    std::cout << "Twist in frame a: ";
    rigid2d::operator<<(std::cout, t);


    // in frame b
    std::cout << "Twist in frame b: ";
    rigid2d::Twist2D tb = Tba.operator()(t);
    rigid2d::operator<<(std::cout, tb);


    // in frame c
    std::cout << "Twist in frame c: ";
    rigid2d::Twist2D tc = Tca.operator()(t);
    rigid2d::operator<<(std::cout, tc);
  }


  else if (frame == 'b')
  {
    // in frame a
    std::cout << "Twist in frame a: ";
    rigid2d::Twist2D ta = Tab.operator()(t);
    rigid2d::operator<<(std::cout, ta);


    // in frame b
    std::cout << "Twist in frame b: ";
    rigid2d::operator<<(std::cout, t);


    // in frame c
    std::cout << "Twist in frame c: ";
    rigid2d::Twist2D tc = Tcb.operator()(t);
    rigid2d::operator<<(std::cout, tc);
  }

  else if (frame == 'c')
  {
    // in frame a
    std::cout << "Twist in frame a: ";
    rigid2d::Twist2D ta = Tac.operator()(t);
    rigid2d::operator<<(std::cout, ta);


    // in frame b
    std::cout << "Twist in frame b: ";
    rigid2d::Twist2D tb = Tbc.operator()(t);
    rigid2d::operator<<(std::cout, tb);


    // in frame c
    std::cout << "Twist in frame c: ";
    rigid2d::operator<<(std::cout, t);
  }
  std::cout << "----------------------" << std::endl;

  return 0;
}


// edn file
