
#include "rigid2d.hpp"
#include <iostream>

int main()
{
  rigid2d::Vector2D v;
  v.x = 1;
  v.y = 2;

  double rad = 3.14;

  rigid2d::Transform2D tf(v, rad);

  rigid2d::Transform2D tf_inv = tf.inv();

  rigid2d::operator<<(std::cout, tf);

  rigid2d::operator<<(std::cout, tf_inv);


  // rigid2d::operator>>(std::cin, v);
  //
  // rigid2d::operator<<(std::cout, v);

  return 0;
}
