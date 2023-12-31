#ifndef ROS2_DDSM115_WHEEL_HPP
#define ROS2_DDSM115_WHEEL_HPP

#include <string>
#include <cmath>


class Wheel
{
    public:

    std::string name = "";
    int id = 0;
    double cmd = 0;
    double pos = 0;
    double vel = 0;

    Wheel() = default;

    Wheel(const std::string &wheel_name, int motor_id)
    {
      setup(wheel_name, motor_id);
    }

    
    void setup(const std::string &wheel_name, int motor_id)
    {
      name = wheel_name;
      id = motor_id;
    }

    double degrees_to_radians(double degrees) 
    {
      return degrees * (M_PI / 180.0);
    }

    double radians_to_degrees(double radians) 
    {
      return radians * (180.0 / M_PI);
    }

    double rpm_to_rad_per_sec(double rpm) 
    {
        return rpm * 0.10472;
    }

    double rad_per_sec_to_rpm(double radPerSec) 
    {
        return (radPerSec * 60.0) / (2.0 * M_PI);
    }

    double mpsToRPM(double mps, double wheelRadius) 
    {
        return (60.0 / (2.0 * M_PI * wheelRadius)) * mps;
    }

    double rpmToMps(double rpm, double wheelRadius) 
    {
        return (2.0 * M_PI * wheelRadius * rpm) / 60.0;
    }

};


#endif // DIFFDRIVE_DDSM115_WHEEL_HPP