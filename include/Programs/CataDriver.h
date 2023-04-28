#pragma once
#include "Programs/CompetitionDriver.h"
#include "pros/misc.h"

class CataDriver : public CompetitionDriver {

public:

    CataDriver(Robot& robot, DRIVE_TYPE driveType):
        CompetitionDriver(robot, driveType), valve('B')
    {}
        
private:

    bool wasLimitSwitchOn = false;
    bool canIntake = true;

    int timeAfterButtonPress;

    pros::ADIDigitalOut valve;

    void initDriver() override;
    void handleSecondaryActions() override;
};