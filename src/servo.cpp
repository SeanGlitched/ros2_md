
#include "ackermann_esp/servo.hpp"

#include <cmath>


Servo::Servo(const std::string &servo_name)
{
  setup(servo_name);
}


void Servo::setup(const std::string &servo_name)
{
  name = servo_name;
}