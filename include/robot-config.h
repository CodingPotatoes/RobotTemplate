#pragma once
#include "vex.h"
#include "core.h"

extern vex::brain brain;
extern vex::controller con;

// ================ INPUTS ================
// Digital sensors

// Analog sensors


// ================ OUTPUTS ================
// Motors
extern vex::motor_group left_motors;
extern vex::motor_group right_motors;
// Pneumatics

// ================ SUBSYSTEMS ================
extern TankDrive driveSus;
// ================ UTILS ================

void robot_init();