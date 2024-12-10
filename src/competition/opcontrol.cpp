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
    DriveTuningConfig tuningConfig(0.7, 0, 24, &drive_pid_cfg, vex::forward, &right_motors, &odom, "Classic PID");
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