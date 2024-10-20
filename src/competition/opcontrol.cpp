#include "competition/opcontrol.h"
#include "vex.h"
#include "robot-config.h"

/**
 * Main entrypoint for the driver control period
*/
void opcontrol()
{
    // ================ INIT ================
while(true){
    double straight = (double)con.Axis3.position() / 100;
    double turn = (double)con.Axis1.position() / 100;

    driveSus.drive_arcade(straight, turn * 0.75, 1);
}


    // ================ PERIODIC ================
}