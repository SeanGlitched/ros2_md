
#include "ackermann_esp/motor.hpp"

#include <cmath>


Motor::Motor(const std::string &motor_name, int counts_per_rev)
{
  setup(motor_name, counts_per_rev);
}


void Motor::setup(const std::string &motor_name, int counts_per_rev)
{
  name = motor_name;
  rads_per_count = (2*M_PI)/counts_per_rev;
}

double Motor::calcEncAngle()
{
  return enc * rads_per_count;
}