#include "competition/opcontrol.h"
#include "vex.h"
#include "robot-config.h"
#include <atomic>
/**
 * Main entrypoint for the driver control period
*/

//Ideally gives us a period of oscillation and max P value to use in the Zeigler-Nichols Method to find PID values
vex::timer stopWatch;
double finalP;
double period;
bool finished = false;

void findPID(string method, PID::pid_config_t thePID, bool driveTurn){
    printf("BadP: %f BadI: %f BadD: %f\n", thePID.p, thePID.i, thePID.d);
    vexDelay(5);
    if(method == "P"){
        printf("Using P: ");
        vexDelay(5);
        thePID.p *= 0.5;
        vexDelay(5);
    }
    else if(method == "PI"){
        printf("Using PI: ");
        vexDelay(5);
        thePID.p *= 0.45;
        vexDelay(5);
        thePID.i = 0.83 * period;
        vexDelay(5);
    }
    else if(method == "PD"){
        printf("Using PD: ");
        vexDelay(5);
        thePID.p *= 0.8;
        vexDelay(5);
        thePID.d = 0.125 * period;
        vexDelay(5);
    }
    else if(method == "Classic PID"){
        printf("Using Classic PID: ");
        vexDelay(5);
        thePID.p *= 0.6;
        vexDelay(5);
        thePID.i = 0.5 * period;
        vexDelay(5);
        thePID.d = 0.125 * period;
        vexDelay(5);
    }
    else if(method == "Pessen Integral"){
        printf("Using Pessen Integral: ");
        vexDelay(5);
        thePID.p *= 0.7;
        vexDelay(5);
        thePID.i = 0.4 * period;
        vexDelay(5);
        thePID.d = 0.15 * period;
        vexDelay(5);
    }
    else if(method == "Some Overshoot"){
        printf("Using Some Overshoot: ");
        vexDelay(5);
        thePID.p *= (1.0 / 3.0);
        vexDelay(5);
        thePID.i = 0.5 * period;
        vexDelay(5);
        thePID.d = (1.0 / 3.0) * period;
        vexDelay(5);
    }
    else if(method == "No Overshoot"){
        printf("Using No Overshoot: ");
        thePID.p *= 0.2;
        thePID.i = 0.5 * period;
        thePID.d = (1.0 / 3.0) * period;
    }
    vexDelay(5);
    printf("GoodP: %f GoodI: %f GoodD: %f\n", thePID.p, thePID.i, thePID.d);
    vexDelay(5);
    if(driveTurn){
        drive_pid_cfg.p = thePID.p;
        vexDelay(5);
        drive_pid_cfg.i = thePID.i;
        vexDelay(5);
        drive_pid_cfg.d = thePID.d;
        vexDelay(5);
    }
    else{
        turn_pid_cfg.p = thePID.p;
        vexDelay(5);
        turn_pid_cfg.i = thePID.i;
        vexDelay(5);
        turn_pid_cfg.d = thePID.d;
        vexDelay(5);
    }
};

class setPIDDrive : public AutoCommand{
    public:
    bool isOscillating = false;
    pose_t initPose;
    bool foundInitPose = false;
    double finalP;
    int initTime;
    double foundInitVelocity = false;
    bool initVelocityPos = true;
    bool isVelocityPos = true;
    bool run() override{
    drive_pid_cfg.p = 0;
    drive_pid_cfg.i = 0;
    drive_pid_cfg.d = 0;
    
    stopWatch.clear();
    printf("Original Time: %d\n", stopWatch.time());

    while(!foundInitVelocity){
        vexDelay(1);
        printf("Velocity: %f, P: %f \n", right_motors.velocity(percent), drive_pid_cfg.p);
        if((right_motors.velocity(percent) > 0.01 || right_motors.velocity(percent) < -0.01)){
            printf("found init velocity\n");
            initVelocityPos = (right_motors.velocity(percent) > 0);
            foundInitVelocity = true;
            vexDelay(1);
        }
        else if((right_motors.velocity(percent) < 0.01 && right_motors.velocity(percent) > -0.01)){
            drive_pid_cfg.p += 0.0005;
            vexDelay(1);
        }
    }
    //Whether or not final values for Max P, Oscillation Period, and Amplitude have been achieved
    while(!finished){
        vexDelay(1);
        // endPose = odom.get_position();
        printf("Time: %d, Position: %f, ", stopWatch.time(), odom.get_position().x);

        if(con.ButtonY.pressing()){
            break;
            return false;
        }

        if((-0.01 >= right_motors.velocity(percent) || right_motors.velocity(percent) >= 0.01)){
        isVelocityPos = (right_motors.velocity(percent) > 0);
        vexDelay(1);
        }

        printf("Init Vel Pos?: %d, Current Vel Pos?: %d \n", initVelocityPos, isVelocityPos);

        if(isOscillating){
            printf("Oscillating...");
        }

        printf("Velocity: %f \n", right_motors.velocity(percent));
        vexDelay(1);
        //if the change in Y >=0 then it either hasnt reached the end point, or hasnt started oscillating, increases P until it starts oscillating
        if((initVelocityPos == isVelocityPos) && !isOscillating && foundInitVelocity){
            drive_pid_cfg.p += 0.002;
            printf("P: %f ", drive_pid_cfg.p);
            vexDelay(1);
            // printf(" P: %f yInit: %f, yEnd: %f, deltaY: %f\n", drive_pid_cfg.p, initPose.y, endPose.y, deltaPose.y);
        }
        //once the change in Y < 0, then it is going backwards and is oscillating, begin finding Period and Amplitude of Oscillation
        else if((initVelocityPos != isVelocityPos) && !isOscillating){
            vexDelay(1);
            isOscillating = true;
            printf(" Oscillation Started\n");
            vexDelay(1);
            finalP = drive_pid_cfg.p;
            initPose = odom.get_position();
            foundInitPose = true;
            isOscillating = true;
            drive_pid_cfg.p *= 1.5;
            initTime = stopWatch.time();
            printf("Pfinal: %f", finalP);
            vexDelay(1);
        }
        else if((initVelocityPos == isVelocityPos) && isOscillating){
            //get the inital position which should be the amplitude of oscillation since it just started oscillating
            //oand the time when the robot is at that position
            //use a vex timer to find the the time when the robot reaches the initial Y value again, the initial time - the final time if your period
            printf("Unchanged P: %f", drive_pid_cfg.p);
            printf("InitTime: %d\n", initTime);
            printf("Final Time: %d ", stopWatch.time());
            period = (((double)stopWatch.time() - (double)initTime)*2) / 1000;
            printf("Ymax: %f , Period: %f\n", initPose.x, period);
            vexDelay(1);
            findPID("Classic PID", drive_pid_cfg, true);
            finished = true;
            vexDelay(1);
            }
        vexDelay(1);
    }
return true;
}
};

class checkError: public AutoCommand{
    public:
    double target;
    bool driveTurn;
    checkError(double target, bool driveTurn) : target(target), driveTurn(driveTurn){

    }
    bool run() override{
        while(true){
            vexDelay(5);
            if(driveTurn){
            printf("Error: %f\n", target - odom.get_position().x);
            }
            else{
                printf("Error: %f\n", target - odom.get_position().rot);
            }
        }
        return true;
    }

};

class finishedCond : public Condition {
    bool test() override {
        return finished;
    }
};

class setPIDTurn : public AutoCommand{
    public:
    bool isOscillating = false;
    pose_t initPose;
    bool foundInitPose = false;
    double finalP;
    int initTime;
    double foundInitVelocity = false;
    bool initVelocityPos = true;
    bool isVelocityPos = true;
    bool run() override{
    turn_pid_cfg.p = 0;
    turn_pid_cfg.i = 0;
    turn_pid_cfg.d = 0;
    
    stopWatch.clear();
    printf("Original Time: %d\n", stopWatch.time());

    while(!foundInitVelocity){
        vexDelay(1);
        printf("Velocity: %f, P: %f \n", imu.gyroRate(zaxis, rpm), turn_pid_cfg.p);
        if((imu.gyroRate(zaxis, rpm) > 0.01 || imu.gyroRate(zaxis, rpm) < -0.01)){
            printf("found init velocity\n");
            initVelocityPos = (imu.gyroRate(zaxis, rpm) > 0);
            foundInitVelocity = true;
            vexDelay(1);
        }
        else if((imu.gyroRate(zaxis, rpm) < 0.01 && imu.gyroRate(zaxis, rpm) > -0.01)){
            turn_pid_cfg.p += 0.0005;
            vexDelay(1);
        }
    }
    //Whether or not final values for Max P, Oscillation Period, and Amplitude have been achieved
    while(!finished){
        vexDelay(1);
        // endPose = odom.get_position();
        printf("Time: %d ", stopWatch.time());

        if(con.ButtonY.pressing()){
            break;
            return false;
        }

        if((-0.01 >= imu.gyroRate(zaxis, rpm) || imu.gyroRate(zaxis, rpm) >= 0.01)){
        isVelocityPos = (imu.gyroRate(zaxis, rpm) > 0);
        vexDelay(1);
        }

        printf("Init Vel Pos?: %d, Current Vel Pos?: %d \n", initVelocityPos, isVelocityPos);

        if(isOscillating){
            printf("Oscillating...");
        }

        printf("Velocity: %f \n", imu.gyroRate(zaxis, rpm));
        vexDelay(1);
        //if the change in Y >=0 then it either hasnt reached the end point, or hasnt started oscillating, increases P until it starts oscillating
        if((initVelocityPos == isVelocityPos) && !isOscillating && foundInitVelocity){
            turn_pid_cfg.p += 0.002;
            printf("P: %f", turn_pid_cfg.p);
            vexDelay(1);
            // printf(" P: %f yInit: %f, yEnd: %f, deltaY: %f\n", drive_pid_cfg.p, initPose.y, endPose.y, deltaPose.y);
        }
        //once the change in Y < 0, then it is going backwards and is oscillating, begin finding Period and Amplitude of Oscillation
        else if((initVelocityPos != isVelocityPos) && !isOscillating){
            vexDelay(1);
            isOscillating = true;
            printf(" Oscillation Started\n");
            vexDelay(1);
            finalP = turn_pid_cfg.p;
            initPose = odom.get_position();
            foundInitPose = true;
            isOscillating = true;
            turn_pid_cfg.p *= 1.5;
            initTime = stopWatch.time();
            printf("Pfinal: %f", finalP);
            vexDelay(1);
        }
        else if((initVelocityPos == isVelocityPos) && isOscillating){
            //get the inital position which should be the amplitude of oscillation since it just started oscillating
            //oand the time when the robot is at that position
            //use a vex timer to find the the time when the robot reaches the initial Y value again, the initial time - the final time if your period
            printf("Unchanged P: %f", turn_pid_cfg.p);
            printf("InitTime: %d\n", initTime);
            printf("Final Time: %d ", stopWatch.time());
            period = (((double)stopWatch.time() - (double)initTime)*2) / 1000;
            printf("Ymax: %f , Period: %f\n", initPose.rot, period);
            vexDelay(1);
            finished = true;
            vexDelay(1);
            }
        vexDelay(5);
    }
    printf("setPID Done");
    return true;
    }
};

void opcontrol()
{
    while(imu.isCalibrating()){
        vexDelay(1);
    }

    
//When X is pressed, print position, and run the command controller

    con.ButtonX.pressed([](){
        for(int i = 100; i > 0;  i--){
            printf("\n");
        }
        printf("Beginning Loop, X: %.2f, Y: %.2f, driveP: %.2f, turnP %.2f ", odom.get_position().x, odom.get_position().y, drive_pid_cfg.p, turn_pid_cfg.p);
        vexDelay(10);
        //Command Controller Runs the SetPID loop parallel to the DriveForward Command
        CommandController cc{
            odom.SetPositionCmd({.x=0,.y=0,.rot=0}),
            new Parallel{
                // driveSus.TurnDegreesCmd(200, 0.7, 0),
                // new setPIDTurn(),
                driveSus.DriveForwardCmd(24, vex::forward, 0.5, 0)->withCancelCondition(new finishedCond()),
                new setPIDDrive()
            }
        };
        
        cc.run();
        
    });
    con.ButtonB.pressed([](){
        printf("Beginning Loop, X: %.2f, Y: %.2f, driveP: %.2f, driveI: %.2f, driveD: %.2f\n turnP: %.2f, turnI: %.2f, turnD: %.2f \n", odom.get_position().x, odom.get_position().y, drive_pid_cfg.p, drive_pid_cfg.i, drive_pid_cfg.d, turn_pid_cfg.p, turn_pid_cfg.i, turn_pid_cfg.d);
        //Command Controller Runs the SetPID loop parallel to the DriveForward Command
        CommandController cc{
            odom.SetPositionCmd({.x=0,.y=0,.rot=0}),
            new Parallel{
                // driveSus.TurnDegreesCmd(200, 0.7, 0),
                driveSus.DriveForwardCmd(24, vex::forward, 0.5, 0),
                new checkError(24, true)
            }
        };
        
        cc.run();
        
    });

    con.ButtonA.pressed([](){
        printf("Beginning Loop, X: %.2f, Y: %.2f, driveP: %.2f, driveI: %.2f, driveD: %.2f\n turnP: %.2f, turnI: %.2f, turnD: %.2f \n", odom.get_position().x, odom.get_position().y, drive_pid_cfg.p, drive_pid_cfg.i, drive_pid_cfg.d, turn_pid_cfg.p, turn_pid_cfg.i, turn_pid_cfg.d);
        //Command Controller Runs the SetPID loop parallel to the DriveForward Command
        CommandController cc{
            odom.SetPositionCmd({.x=0,.y=0,.rot=0}),
            new Parallel{
                // driveSus.TurnDegreesCmd(200, 0.7, 0),
                driveSus.DriveForwardCmd(12, vex::reverse, 0.5, 0),
                new checkError(-12, true)
            }
        };
        
        cc.run();
        
    });


    // ================ INIT ================

while(true){
    vexDelay(5);
    double straight = (double)con.Axis3.position() / 100;
    double turn = (double)con.Axis1.position() / 100;
}


    // ================ PERIODIC ================
}