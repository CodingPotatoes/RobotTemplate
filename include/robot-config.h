#pragma once
#include "vex.h"
#include "core.h"

extern vex::brain brain;
extern vex::controller con;

// ================ INPUTS ================
// Digital sensors
extern vex::inertial imu;
// Analog sensors
extern OdometryTank odom;
// ================ OUTPUTS ================
// Motors
extern vex::motor_group left_motors;
extern vex::motor_group right_motors;

// Pneumatics
extern PID turn_pid;
extern PID drive_pid;
extern PID::pid_config_t drive_pid_cfg;
extern PID::pid_config_t turn_pid_cfg;
// ================ SUBSYSTEMS ================
extern TankDrive driveSus;
// ================ UTILS ================

void robot_init();