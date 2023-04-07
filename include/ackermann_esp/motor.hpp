#pragma once
#include <string>



class Motor
{
    public:

    std::string name = "";
    int enc = 0;
    double cmd = 0;
    double pos = 0;
    double vel = 0;
    double eff = 0;
    double velSetPt = 0;
    double rads_per_count = 0;

    Motor() = default;

    Motor(const std::string &motor_name, int counts_per_rev);
    
    void setup(const std::string &motor_name, int counts_per_rev);

    double calcEncAngle();



};