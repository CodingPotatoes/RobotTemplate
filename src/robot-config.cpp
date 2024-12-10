#include "robot-config.h"
#include "../core/include/subsystems/odometry/odometry_nwheel.h"

vex::brain Brain;
vex::controller con;

// ================ INPUTS ================
// Digital sensors

// Analog sensors
CustomEncoder left_enc{Brain.ThreeWirePort.C, 2048};
CustomEncoder right_enc{Brain.ThreeWirePort.E, 2048};
CustomEncoder front_enc{Brain.ThreeWirePort.G, 2048};

// ================ OUTPUTS ================
// Motors
vex::motor left_front_top(vex::PORT8, vex::gearSetting::ratio6_1, true);
vex::motor left_front_bottom(vex::PORT6, vex::gearSetting::ratio6_1, true);
vex::motor left_back_top(vex::PORT7, vex::gearSetting::ratio6_1, false);
vex::motor left_back_bottom(vex::PORT5, vex::gearSetting::ratio6_1, true);

vex::motor right_front_top(vex::PORT3, vex::gearSetting::ratio6_1, false);
vex::motor right_front_bottom(vex::PORT2, vex::gearSetting::ratio6_1, false);
vex::motor right_back_top(vex::PORT1, vex::gearSetting::ratio6_1, true);
vex::motor right_back_bottom(vex::PORT4, vex::gearSetting::ratio6_1, false);

// ================ SUBSYSTEMS ================
vex::motor_group left_motors{left_front_top, left_front_bottom, left_back_top, left_back_bottom};
vex::motor_group right_motors{right_front_top, right_front_bottom, right_back_top, right_back_bottom};



// ======== SUBSYSTEMS ========
PID::pid_config_t drive_pid_cfg{
    //0.543480 GoodI: 0.498000 GoodD: 0.124500
    .p = 0.
    ,.i = 0.
    ,.d = 0.
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

robot_specs_t robot_cfg{
    .robot_radius = 12.0,
    .odom_wheel_diam = 2.125,
    .odom_gear_ratio = 1.0,
    .dist_between_wheels = 11.0,

    // .drive_correction_cutoff = 0,
    .drive_feedback = &drive_pid,
    .turn_feedback = &turn_pid,
};

// Odometry

vex::inertial imu(vex::PORT14, vex::turnType::right);

tracking_wheel_cfg_t left_config{-1.75, 2.25, 0, 1.0625};
tracking_wheel_cfg_t right_config{1.75, -2.25, M_PI, 1.0625};
tracking_wheel_cfg_t rear_config{-8.5, -0.5, ((3 * M_PI) / 2), 1.0625};

// OdometryNWheel<3> odom({left_encoder, right_encoder, rear_encoder}, {left_config, right_config, rear_config}, &imu, true);

OdometryTank odom(left_enc, right_enc, robot_cfg, &imu);
TankDrive driveSus(left_motors, right_motors, robot_cfg, &odom);

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