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

class setPIDDrive : public AutoCommand{
    public:
    bool run() override{
    drive_pid_cfg.p =0;
    drive_pid_cfg.i =0;
    drive_pid_cfg.d =0;
    bool startedOscillation = false;
    pose_t initPose;
    bool finished = false;
    bool foundInitPose = false;
    double finalP;
    int initTime;
    stopWatch.clear();
    printf("Original Time: %d\n", stopWatch.time());
    //Whether or not final values for Max P, Oscillation Period, and Amplitude have been achieved
    while(!finished){
        vexDelay(5);
        // endPose = odom.get_position();
        printf("Time: %d ", stopWatch.time());

        if(con.ButtonY.pressing()){
            break;
        }
        printf("Velocity: %f \n", right_motors.velocity(percent));

        //if the change in Y >=0 then it either hasnt reached the end point, or hasnt started oscillating, increases P until it starts oscillating
        if((right_motors.velocity(percent) >= 0 ) && startedOscillation == false){
            drive_pid_cfg.p += 0.005;
            printf("P: %f", drive_pid_cfg.p);
            // printf(" P: %f yInit: %f, yEnd: %f, deltaY: %f\n", drive_pid_cfg.p, initPose.y, endPose.y, deltaPose.y);
        }
        //once the change in Y < 0, then it is going backwards and is oscillating, begin finding Period and Amplitude of Oscillation
        else{
            startedOscillation = true;
            drive_pid_cfg.p *= 1.1;
            printf(" Oscillation Started\n");
            if(foundInitPose == false){
                finalP = drive_pid_cfg.p;
                initPose = odom.get_position();
                foundInitPose = true;
                startedOscillation = true;
            initTime = stopWatch.time();
            printf("Pfinal: %f", finalP);
            }
            
            
            //get the inital position which should be the amplitude of oscillation since it just started oscillating
            //oand the time when the robot is at that position
            //use a vex timer to find the the time when the robot reaches the initial Y value again, the initial time - the final time if your period
            printf("Unchanged P: %f", drive_pid_cfg.p);
            if(left_motors.velocity(percent) > 0){
                printf("InitTime: %d\n", initTime);
                printf("Final Time: %d ", stopWatch.time());
                period = (((double)stopWatch.time() - (double)initTime)*2) / 1000;
                printf("Ymax: %f , Period: %f\n", initPose.y, period);

                findPID("PID");
                printf("GoodP: %f GoodI: %f GoodD: %f", drive_pid_cfg.p, drive_pid_cfg.i, drive_pid_cfg.d);
                vexDelay(5);
                finished = true;
                return true;
            }
        }
    }
    return false;
    }
};

class checkErrorDrive: public AutoCommand{
    public:
    bool run() override{
        while(true){
            printf("Error: %f\n", 24 - odom.get_position().x);
        }
        return true;
    }

};

class checkErrorTurn: public AutoCommand{
    public:
    bool run() override{
        while(true){
            printf("Error: %f\n", 200 - odom.get_position().rot);
        }
        return true;
    }

};

void findPID(string method){
    if(method == "P"){
        drive_pid_cfg.p *= 0.5;
    }
    else if(method == "PI"){
        drive_pid_cfg.p *= 0.45;
        drive_pid_cfg.i *= 0.83 * period;
    }
    else if(method == "PD"){
        drive_pid_cfg.p *= 0.8;
        drive_pid_cfg.d = 0.125 * period;
    }
    else if(method == "Classic PID"){
        drive_pid_cfg.p *= 0.6;
        drive_pid_cfg.i = 0.5 * period;
        drive_pid_cfg.d = 0.125 * period;
    }
    else if(method == "Pessen Integral"){
        drive_pid_cfg.p *= 0.7;
        drive_pid_cfg.i = 0.4 * period;
        drive_pid_cfg.d = 0.15 * period;
    }
    else if(method == "Some Overshoot"){
        drive_pid_cfg.p *= (1/3);
        drive_pid_cfg.i = 0.5 * period;
        drive_pid_cfg.d = (1/3) * period;
    }
    else if(method == "No Overshoot"){
        drive_pid_cfg.p *= 0.2;
        drive_pid_cfg.i = 0.5 * period;
        drive_pid_cfg.d = (1/3) * period;
    }
}


class SetPIDTurnFindP : public AutoCommand{
    public:
    bool run() override{
    //initiates values for turning PID
    bool finished = false;
    bool foundInitVelocity = false;
    bool initVelocityPos = true;
    bool isVelocityPos = true;
    for(int i = 3; i > 0;  i--){
            printf("\n");
        }
    printf("Find P Started, Original Time: %d\n", stopWatch.time());
    //Whether or not final values for Max P, Oscillation Period, and Amplitude have been achieved
    while(!foundInitVelocity){
        printf("GyroRate: %f, P: %f \n", imu.gyroRate(zaxis, rpm), turn_pid_cfg.p);
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

    while(!finished){
        if(con.ButtonY.pressing()){
            break;
        }

        printf("Velocity: %f, IMU Rotation: %f, odometry position: %f \n", imu.gyroRate(zaxis, rpm), imu.heading(degrees), odom.get_position().rot);
        vexDelay(1);
        printf("Time: %d ", stopWatch.time());

        if((-0.01 >= imu.gyroRate(zaxis, rpm) >= 0.01)){
        isVelocityPos = (imu.gyroRate(zaxis, rpm) > 0);
        }
        printf("Init Vel Pos?: %d, Current Vel Pos?: %d \n", initVelocityPos, isVelocityPos);
        // vexDelay(2);

        if((initVelocityPos == isVelocityPos)){
            turn_pid_cfg.p += 0.0005;
            printf("P: %f ", turn_pid_cfg.p);
        }
        else{
            
            printf(" Oscillation Started\n");
            printf("Velocity: %f, IMU Rotation: %f, odometry position: %f \n", imu.gyroRate(zaxis, rpm), imu.heading(degrees), odom.get_position().rot);
            // turn_pid_cfg.p *= 1.4;
            finalP = turn_pid_cfg.p;
            printf("Pfinal: %f\n", finalP);
            return true;
        }
    }
    return false;
    }
};

class setPIDTurnOscillating : public AutoCommand{
    public:
    bool run() override{
    //initiates values for turning PID
    pose_t initPose;
    bool finished = false;
    bool foundInitPose = false;
    bool isOscillating = false;
    int initTime;
    stopWatch.clear();
    bool foundInitVelocity = false;
    bool initVelocityPos;
    bool isVelocityPos;
    for(int i = 3; i > 0;  i--){
            printf("\n");
        }
        // vexDelay(3000);
    printf("Find Period Started, Original Time: %d\n", stopWatch.time());
    //Whether or not final values for Max P, Oscillation Period, and Amplitude have been achieved
    while(!foundInitVelocity){
        if((imu.gyroRate(zaxis, rpm) > 0.01 || imu.gyroRate(zaxis, rpm) < -0.01)){
            vexDelay(1);
            printf("found init velocity\n");
            initVelocityPos = (imu.gyroRate(zaxis, rpm) > 0);
            foundInitVelocity = true;
        }
    vexDelay(1);
    }
    vexDelay(1);
    
    while(!finished){
        if(con.ButtonY.pressing()){
            break;
        }
        vexDelay(1);
        printf("Velocity: %f, IMU Rotation: %f, odometry position: %f, ", imu.gyroRate(zaxis, rpm), imu.heading(degrees), odom.get_position().rot);
        printf("Time: %d ", stopWatch.time());
        vexDelay(1);
        
        if((imu.gyroRate(zaxis, rpm) > 0.01 || imu.gyroRate(zaxis, rpm) < -0.01)){
        isVelocityPos = imu.gyroRate(zaxis, rpm) >= 0;
        vexDelay(1);
        }
        if(isOscillating){
            printf("Currently Oscillating... ");
            vexDelay(1);
        }
        vexDelay(1);

        printf("Init Vel Pos?: %d, Current Vel Pos?: %d \n", initVelocityPos, isVelocityPos);
        vexDelay(1);

        //if the change in Y >=0 then it either hasnt reached the end point, or hasnt started oscillating, increases P until it starts oscillating
        if((isVelocityPos != initVelocityPos) && !isOscillating && foundInitVelocity){
            printf("Velocity: %f, IMU Rotation: %f, odometry position: %f \n", imu.gyroRate(zaxis, rpm), imu.heading(degrees), odom.get_position().rot);
            printf("Begun Oscillating\n");
            vexDelay(1);
            isOscillating = true;
            if(foundInitPose == false){
                initPose = odom.get_position();
                foundInitPose = true;
                initTime = stopWatch.time();
                vexDelay(1);
            }
            vexDelay(1);
        }
        else if((isVelocityPos == initVelocityPos) && isOscillating){
            vexDelay(1);
            printf("Oscillation Completed\n");
            printf("InitTime: %d\n", initTime);
            printf("Final Time: %d ", stopWatch.time());

            period = (((double)stopWatch.time() - (double)initTime)*2) / 1000;
            printf("RotMax: %f , Period: %f\n", initPose.rot, period);
            vexDelay(1);

            findPID("P");
            printf("GoodP: %f GoodI: %f GoodD: %f\n", drive_pid_cfg.p, drive_pid_cfg.i, drive_pid_cfg.p);
            vexDelay(1);
            finished = true;
        }
        vexDelay(1);
    }
    return false;
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
        //Command Controller Runs the SetPID loop parallel to the DriveForward Command
        CommandController cc{
            odom.SetPositionCmd({.x=0,.y=0,.rot=0}),
            new Parallel{
                // new setPIDDrive(),
                new SetPIDTurnFindP(),
                driveSus.TurnDegreesCmd(200, 0.7, 0),
                // driveSus.DriveForwardCmd(24, vex::reverse, 0.7, 0),
            },
            new DelayCommand(5000),
            odom.SetPositionCmd({.x=0,.y=0,.rot=0}),
            new Parallel{
                // new setPIDDrive(),
                new setPIDTurnOscillating(),
                driveSus.TurnDegreesCmd(200, 0.7, 0),
                // driveSus.DriveForwardCmd(24, vex::reverse, 0.7, 0),
            },
        };
        
        cc.run();
        
    });
    con.ButtonB.pressed([](){
        printf("Beginning Loop, X: %.2f, Y: %.2f, driveP: %.2f, turnP %.2f ", odom.get_position().x, odom.get_position().y, drive_pid_cfg.p, turn_pid_cfg.p);
        //Command Controller Runs the SetPID loop parallel to the DriveForward Command
        CommandController cc{
            odom.SetPositionCmd({.x=0,.y=0,.rot=0}),
            new Parallel{
                driveSus.TurnDegreesCmd(200, 0.7, 0),
                new checkErrorTurn,
                // driveSus.DriveForwardCmd(24, vex::forward, 0.7, 0),
                // new checkErrorDrive(),
            },
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