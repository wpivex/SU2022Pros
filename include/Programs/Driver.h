#pragma once
#include "misc/ControllerSM.h"
#include "Subsystems/Robot.h"


class Driver {

protected:

    Robot& robot;

public:

    Driver(Robot& robot): robot(robot) {}
    virtual void runDriver() = 0;

    ControllerSM controller;
        
};