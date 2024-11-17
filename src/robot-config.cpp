#include "robot-config.h"

vex::brain Brain;
vex::controller con;

// ================ INPUTS ================
// Digital sensors

// Analog sensors

// ================ OUTPUTS ================
// Motors
vex::motor left_front(vex::PORT20, vex::gearSetting::ratio6_1, true);
vex::motor left_middle(vex::PORT19, vex::gearSetting::ratio6_1, true);
vex::motor left_rear(vex::PORT18, vex::gearSetting::ratio6_1, true);
vex::motor_group left_motors = {left_front, left_middle, left_rear};

vex::motor right_front(vex::PORT13, vex::gearSetting::ratio6_1, false);
vex::motor right_middle(vex::PORT12, vex::gearSetting::ratio6_1, false);
vex::motor right_rear(vex::PORT11, vex::gearSetting::ratio6_1, false);
vex::motor_group right_motors = {right_front, right_middle, right_rear};

// ================ SUBSYSTEMS ================




// ======== SUBSYSTEMS ========
PID::pid_config_t drive_pid_cfg{
    .p = 0.2805//0.2871
    ,.i = 0.134//0.099
    ,.d = 0.0335//0.02475
    ,.deadband = 0.5
    ,.on_target_time = 0.06
};

PID drive_pid(drive_pid_cfg);

PID::pid_config_t turn_pid_cfg{
    //.p = 0.02,//
    .p = 0,
    //.p = 0,
    .i = 0,
    .d = 0,
    //.d = 0.002,
    .deadband = 0.5,
    .on_target_time = 0.06
};
PID turn_pid(turn_pid_cfg);

robot_specs_t robot_cfg = {
    .robot_radius  = 10,
    .odom_wheel_diam = 3.0,
    .odom_gear_ratio = (4.0 / 3.0),
    .dist_between_wheels = 5.5,

    .drive_feedback = &drive_pid,
    .turn_feedback = &turn_pid,
};

TankDrive driveSus(left_motors, right_motors, robot_cfg, &odom);

// Odometry
CustomEncoder left_enc(Brain.ThreeWirePort.B, 2048);
CustomEncoder right_enc(Brain.ThreeWirePort.C, 2048);

vex::inertial imu(vex::PORT14, vex::turnType::right);

OdometryTank odom(left_motors, right_motors,robot_cfg, &imu);

vex::controller::button setPosition = con.ButtonX;

// ================ UTILS ================

OdometryTank& realOdom = odom;


/**
 * Main robot initialization on startup. Runs before opcontrol and autonomous are started.
 */
void robot_init()
{
    imu.startCalibration();
    screen::start_screen(Brain.Screen, 
    {
        new screen::PIDPage(drive_pid, "PID DEEZ NUTS"),
        new screen::PIDPage(turn_pid, "STUN ZEED DIP"),
        new::screen::OdometryPage(odom, 15, 15, true),
    });

}