#include "competition/opcontrol.h"
#include "vex.h"
#include "robot-config.h"
#include <atomic>
#include "../core/include/utils/AutoTuningPID.h"

/**
 * Main entrypoint for the driver control period
*/

void opcontrol()
{
    while(imu.isCalibrating()){
        vexDelay(1);
    }
    static DriveTuningConfig tuningConfig{
        .startspeed = 0.5
    ,.endspeed = 0
    ,.dir = vex::forward
    ,.distance = 24
    ,.pidConfig = &drive_pid_cfg
    ,.right_motors = &right_motors
    ,.odom = odom
    };
    static AutoTuningTools autoTuner(&driveSus, tuningConfig);
    con.ButtonX.pressed([](){
        autoTuner.TuneDrivePID();
    });
    con.ButtonB.pressed([](){
        autoTuner.driveWithError();
});

    con.ButtonA.pressed([](){
        autoTuner.driveWithError();
});


    // ================ INIT ================

while(true){
    vexDelay(5);
    double straight = (double)con.Axis3.position() / 100;
    double turn = (double)con.Axis1.position() / 100;
}


    // ================ PERIODIC ================
}