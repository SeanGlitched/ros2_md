#pragma once
#include <string>



class Servo
{
    public:

    std::string name = "";
    int enc = 0;
    double cmd = 0;
    double pos = 0;
    double eff = 0;
    double velSetPt = 0;
    double rads_per_count = 0;

    Servo() = default;

    Servo(const std::string &motor_name);
    
    void setup(const std::string &motor_name);




};