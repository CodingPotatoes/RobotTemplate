#include "robot-config.h"
#include "../core/include/subsystems/odometry/odometry_nwheel.h"

vex::brain Brain;
vex::controller con;

// ================ INPUTS ================
// Digital sensors

// Analog sensors
CustomEncoder left_encoder{Brain.ThreeWirePort.A, 2048};
CustomEncoder right_encoder{Brain.ThreeWirePort.C, 2048};
CustomEncoder rear_encoder{Brain.ThreeWirePort.E, 2048};

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
    .p = 0.40362
    ,.i = 0.09500
    ,.d = 0.02375
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
    .dist_between_wheels = 9.875,

    .drive_feedback = &drive_pid,
    .turn_feedback = &turn_pid,
};

TankDrive driveSus(left_motors, right_motors, robot_cfg, &odom);

// Odometry
CustomEncoder left_enc(Brain.ThreeWirePort.B, 2048);
CustomEncoder right_enc(Brain.ThreeWirePort.C, 2048);

vex::inertial imu(vex::PORT14, vex::turnType::right);

tracking_wheel_cfg_t left_config{-1.75, 2.25, 0, 1.0625};
tracking_wheel_cfg_t right_config{1.75, -2.25, M_PI, 1.0625};
tracking_wheel_cfg_t rear_config{-8.5, -0.5, ((3 * M_PI) / 2), 1.0625};

// OdometryNWheel<3> odom({left_encoder, right_encoder, rear_encoder}, {left_config, right_config, rear_config}, &imu, true);

OdometryTank odom(left_encoder, right_encoder, robot_cfg, &imu);

vex::controller::button setPosition = con.ButtonX;

// ================ UTILS ================

// OdometryTank& realOdom = odom;


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