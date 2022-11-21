#pragma once

#include "misc/ButtonSM.h"
#include "Subsystems/Robot.h"
#include "pros/misc.h"

class Driver {

public:

    Driver(Robot& robotReference):
        robot(robotReference)
    {}

    void runDriver();
        
private:

    void handleDrivetrain();
    void handleSecondaryActions();

    ButtonSM buttonSM;
    Robot& robot;

    bool indexerOn;
    int indexerTimer;
};