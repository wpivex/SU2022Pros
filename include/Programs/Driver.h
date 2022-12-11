#pragma once

#include "misc/ControllerSM.h"
#include "Subsystems/Robot.h"
#include "pros/misc.h"


enum DRIVE_TYPE {TANK_DRIVE, ARCADE_DRIVE};

class Driver {

public:

    Driver(Robot& robotReference, DRIVE_TYPE driveType):
        robot(robotReference),
        drive(driveType)
    {}

    void runDriver();
        
private:

    void handleDrivetrain();
    void handleSecondaryActions();

    ControllerSM controller;
    Robot& robot;

    DRIVE_TYPE drive;

    bool indexerOn;
    int indexerTimer, indexerOffTimer;
    int speed = 3000;
};