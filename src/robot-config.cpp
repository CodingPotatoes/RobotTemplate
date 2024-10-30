#include "robot-config.h"

vex::brain Brain;
vex::controller con;

// ================ INPUTS ================
// Digital sensors

// Analog sensors

// ================ OUTPUTS ================
// Motors
vex::motor left_front(vex::PORT11, vex::gearSetting::ratio36_1, true);
vex::motor left_middle(vex::PORT12, vex::gearSetting::ratio36_1, true);
vex::motor left_rear(vex::PORT13, vex::gearSetting::ratio36_1, true);
vex::motor_group left_motors = {left_front, left_middle, left_rear};

vex::motor right_front(vex::PORT18, vex::gearSetting::ratio36_1, false);
vex::motor right_middle(vex::PORT19, vex::gearSetting::ratio36_1, false);
vex::motor right_rear(vex::PORT20, vex::gearSetting::ratio36_1, false);
vex::motor_group right_motors = {right_front, right_middle, right_rear};

// ================ SUBSYSTEMS ================





// ======== SUBSYSTEMS ========
PID::pid_config_t drive_pid_cfg{};
PID drive_pid(drive_pid_cfg);

PID::pid_config_t turn_pid_cfg{};
PID turn_pid(turn_pid_cfg);

robot_specs_t robot_cfg = {
    .robot_radius  = 12.0,
    .odom_wheel_diam = 2.0,
    .odom_gear_ratio = 1.0,
    .dist_between_wheels = 10.0,

    .drive_feedback = &drive_pid,
    .turn_feedback = &turn_pid,
};

TankDrive driveSus(left_motors, right_motors, robot_cfg, &odom);


// Odometry
CustomEncoder left_enc(Brain.ThreeWirePort.B, 2048);
CustomEncoder right_enc(Brain.ThreeWirePort.C, 2048);

vex::inertial imu(vex::PORT14, vex::turnType::right);

OdometryTank odom(left_enc, right_enc, robot_cfg, &imu);

vex::controller::button setPosition = con.ButtonX;

// ================ UTILS ================




/**
 * Main robot initialization on startup. Runs before opcontrol and autonomous are started.
 */
void robot_init()
{
    imu.startCalibration();
    vexDelay(1);
}