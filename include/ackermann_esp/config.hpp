#pragma once

#include <cstring>

struct Config
{
  float loop_rate = 30;
  std::string device = "/dev/ttyUSB0";
  int baud_rate = 115200;
  int timeout = 1000;
  int enc_counts_per_rev = 3600;
};
