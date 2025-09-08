#ifndef WHEEL_HPP
#define WHEEL_HPP

#include <string>

class Wheel
{
public:
  std::string name = "";
  int wheel_diameter = 0;
  double pos = 0.0;
  double vel = 0.0;
  double cmd = 0.0;

  void setup(const std::string& wheel_name, int diameter)
  {
    name = wheel_name;
    wheel_diameter = diameter;
  }
};

#endif
