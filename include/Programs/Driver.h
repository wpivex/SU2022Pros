#pragma once
#include "Algorithms/Alternator.h"
#include "misc/ControllerSM.h"
#include "Subsystems/Robot.h"
#include "pros/misc.h"
#include "Algorithms/Shooter.h"

enum DRIVE_TYPE {TANK_DRIVE, ARCADE_DRIVE};

class Driver {

public:

    Driver(Robot& robotReference, DRIVE_TYPE driveType, int defaultFlywheelSpeed):
        robot(robotReference),
        drive(driveType),
        shootAlternator(3, 11, 7), // 70ms on / 70ms off
        DEFAULT_SPEED(defaultFlywheelSpeed)
    {
        speed = DEFAULT_SPEED;
    }

    void runDriver();

    ControllerSM controller;
        
private:

    const int DEFAULT_SPEED;

    void handleDrivetrain();
    void handleSecondaryActions();

    Robot& robot;

    DRIVE_TYPE drive;

    Alternator shootAlternator;

    Shooter shooter;

    bool indexerOn;
    bool flapUp = true;
    int indexerTimer, indexerOffTimer;
    
    int speed;

    double shootSpeed = 1;

};