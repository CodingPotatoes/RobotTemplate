#include "competition/opcontrol.h"
#include "vex.h"
#include "robot-config.h"

/**
 * Main entrypoint for the driver control period
*/
void opcontrol()
{

    pose_t posooton;

    while(imu.isCalibrating()){
        vexDelay(1);
    }
    // ================ INIT ================
while(true){
    double straight = (double)con.Axis3.position() / 100;
    double turn = (double)con.Axis1.position() / 100;

    driveSus.drive_arcade(straight, turn * 0.75, 1);
    if(con.ButtonX.PRESSED){
        odom.set_position({.x=0, .y=0, .rot=0});
    }
    posooton = odom.get_position();

    printf("X: %.2f, Y: %.2f", posooton.x, posooton.y);

}


    // ================ PERIODIC ================
}