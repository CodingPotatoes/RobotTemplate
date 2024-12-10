#pragma once
#include "../core/include/subsystems/odometry/odometry_tank.h"
#include "../core/include/subsystems/tank_drive.h"
#include <string>


struct DriveTuningConfig{
    double startspeed;
    double endspeed;
    int distance;
    PID::pid_config_t &pidConfig;
    vex::directionType dir;
    vex::motor_group &right_motors;
    OdometryTank &odom;
    std::string tunerMethod;
    DriveTuningConfig(double startspeed, double endspeed, int distance, PID::pid_config_t *pidConfig, vex::directionType dir, vex::motor_group *right_motors, OdometryTank *odom,std::string tunerMethod)
     : startspeed(startspeed), endspeed(endspeed), distance(distance), pidConfig(*pidConfig), dir(dir), right_motors(*right_motors), odom(*odom), tunerMethod(tunerMethod){};
};
class AutoTuningTools{
    public:
    TankDrive &driveSys;
    DriveTuningConfig cfg;
    AutoTuningTools(TankDrive *driveSys, DriveTuningConfig cfg) : driveSys(*driveSys), cfg(cfg){};
    void TuneDrivePID();
    void driveWithError();
};