#pragma once

#include <string>

struct Config
{
  //float loop_rate = 30;
  std::string device = "/dev/ttyUSB0";
  std::string motor_name = "back_axel";
  std::string servo_name = "front_joint";
  int baud_rate = 115200;
  int timeout = 1000;
  int enc_counts_per_rev = 3600;
};
