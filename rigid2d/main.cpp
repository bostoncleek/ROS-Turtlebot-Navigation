
#include "rigid2d.hpp"
#include <iostream>

int main()
{

  // inputs
  rigid2d::Transform2D Tab;
  rigid2d::Transform2D Tbc;
  rigid2d::Vector2D v;
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


  // vector in desired frame
  std::cout << "Enter a vector"<< std::endl;
  rigid2d::operator>>(std::cin, v);

  std::cout << "Enter a character defining the frame the vector is in: 'a', 'b', 'c'" << std::endl;
  std::cin >> frame;






  // rigid2d::Vector2D v1;
  // v1.x = 1;
  // v1.y = 2;
  //
  // // rigid2d::Vector2D v2;
  // // v2.x = 2;
  // // v2.y = 2;
  //
  // double rad1 = rigid2d::PI/2;
  // // double rad2 = 0.0;//rigid2d::PI;
  //
  //
  // rigid2d::Transform2D tf_1(v1, rad1);
  // // rigid2d::Transform2D tf_2(v2, rad2);
  //
  // rigid2d::Transform2D tf_inv = tf_1.inv();
  // //
  // rigid2d::operator<<(std::cout, tf_1);
  // // rigid2d::operator<<(std::cout, tf_2);
  //
  // //
  // rigid2d::operator<<(std::cout, tf_inv);

  //rigid2d::operator>>(std::cin, tf_1); //std::istream & is, Transform2D & tf)

  // rigid2d::Transform2D tf_res = rigid2d::operator*(tf_1, tf_2);
  //
  // rigid2d::operator<<(std::cout, tf_res);

  // rigid2d::operator>>(std::cin, v);
  //
  // rigid2d::operator<<(std::cout, v);

  return 0;
}
