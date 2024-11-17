#include "competition/opcontrol.h"
#include "vex.h"
#include "robot-config.h"
#include <atomic>
/**
 * Main entrypoint for the driver control period
*/

//Ideally gives us a period of oscillation and max P value to use in the Zeigler-Nichols Method to find PID values
vex::timer stopWatch;
int initTime;
class setPIDDrive : public AutoCommand{
    public:
    bool run() override{
    drive_pid_cfg.p =0;
    drive_pid_cfg.i =0;
    drive_pid_cfg.d =0;
    //initiates values for turning PID
    bool startedOscillation = false;
    // int deltaY;
    // int yInit;
    // int yEnd;
    pose_t initPose;
    pose_t endPose;
    pose_t deltaPose;
    bool finished = false;
    bool foundInitPose = false;
    double finalP;
    // int initTime;
    double period;
    // vex::timer stopWatch;
    stopWatch.clear();
    printf("Original Time: %d\n", stopWatch.time());
    //Whether or not final values for Max P, Oscillation Period, and Amplitude have been achieved
    while(!finished){
        // odom.get_speed();
        // printf("setPID is running, Y is %f\n", odom.get_position().y);
        //gets initial Y position
        // initPose = odom.get_position();
        // printf("Yinit: %f", initPose.y);
        //waits 20ms and gets final Y position and calculates the change
        vexDelay(5);
        // endPose = odom.get_position();
        // deltaPose.y = endPose.y - initPose.y;
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

                double goodP = finalP * 0.6;
                double goodI = 0.5 * (period);
                double goodD = 0.125 * (period);
                printf("GoodP: %f GoodI: %f GoodD: %f", goodP, goodI, goodD);
                drive_pid_cfg.p =goodP;
                drive_pid_cfg.i =goodI;
                drive_pid_cfg.d =goodD;
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
            printf("Error: %f\n", 180 - odom.get_position().rot);
        }
        return true;
    }

};


class setPIDRot : public AutoCommand{
    public:
    bool run() override{
    //initiates values for turning PID
    bool startedOscillation = false;
    pose_t initPose;
    bool finished = false;
    bool foundInitPose = false;
    double finalP;
    double period;
    stopWatch.clear();
    printf("Original Time: %d\n", stopWatch.time());
    //Whether or not final values for Max P, Oscillation Period, and Amplitude have been achieved
    while(!finished){
        printf("Velocity: %f, IMU Rotation: %f, odometry position: %f \n", -imu.gyroRate(zaxis, rpm), imu.heading(degrees), odom.get_position().rot);
        // printf("setPID is running, Y is %f\n", odom.get_position().y);
        //gets initial Y position
        // printf("Yinit: %f", initPose.y);
        //waits 20ms and gets final Y position and calculates the change
        vexDelay(5);
        // endPose = odom.get_position();
        // deltaPose.y = endPose.y - initPose.y;
        printf("Time: %d ", stopWatch.time());

        if(con.ButtonY.pressing()){
            break;
        }
        // printf("Velocity: %f, IMU Rotation: %f, odometry position: %f \n", -imu.gyroRate(zaxis, dps), imu.heading(degrees), odom.get_position().rot);
        vexDelay(2);

        //if the change in Y >=0 then it either hasnt reached the end point, or hasnt started oscillating, increases P until it starts oscillating
        if((-imu.gyroRate(zaxis, rpm) >= -0.001 ) && startedOscillation == false){
            turn_pid_cfg.p += 0.005;
            printf("P: %f ", turn_pid_cfg.p);
            vexDelay(2);
            // printf(" P: %f yInit: %f, yEnd: %f, deltaY: %f\n", drive_pid_cfg.p, initPose.y, endPose.y, deltaPose.y);
        }
        //once the change in Y < 0, then it is going backwards and is oscillating, begin finding Period and Amplitude of Oscillation
        else{
            startedOscillation = true;
            
            printf(" Oscillation Started\n");
            if(foundInitPose == false){
                turn_pid_cfg.p *= 1.1;
                finalP = turn_pid_cfg.p;
                initPose = odom.get_position();
                foundInitPose = true;
            initTime = stopWatch.time();
            printf("Pfinal: %f\n", finalP);
            }
            
            
            //get the inital position which should be the amplitude of oscillation since it just started oscillating
            //oand the time when the robot is at that position
            //use a vex timer to find the the time when the robot reaches the initial Y value again, the initial time - the final time if your period
            printf("Unchanged P: %f\n", turn_pid_cfg.p);
            if(-imu.gyroRate(zaxis, rpm) >= -0.001){
                printf("InitTime: %d\n", initTime);
                printf("Final Time: %d ", stopWatch.time());
                period = (((double)stopWatch.time() - (double)initTime)*2) / 1000;
                printf("RotMax: %f , Period: %f\n", initPose.rot, period);

                double GoodP = finalP * 0.6;
                double GoodI = 0.5 * (period);
                double GoodD = 0.125 * (period);
                printf("GoodP: %f GoodI: %f GoodD: %f\n", GoodP, GoodI, GoodD);
                turn_pid_cfg.p = GoodP;
                turn_pid_cfg.i = GoodI;
                turn_pid_cfg.d = GoodD;
                vexDelay(5);
                finished = true;
                return true;
            }
        }
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
                new setPIDRot(),
                driveSus.TurnDegreesCmd(180, 0.4, 0),
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
                driveSus.TurnDegreesCmd(180, 0.7, 0),
                new checkErrorTurn,
                // driveSus.DriveForwardCmd(24, vex::forward, 0.7, 0),
                // new checkErrorDrive(),
            },
        };
        
        cc.run();
        
    });


    // ================ INIT ================
    int deltaY;
    int yInit;
    int yEnd;
    pose_t initPose;
    pose_t endPose;
    pose_t deltaPose;

while(true){
    vexDelay(5);
    double straight = (double)con.Axis3.position() / 100;
    double turn = (double)con.Axis1.position() / 100;
}


    // ================ PERIODIC ================
}