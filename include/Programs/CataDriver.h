#pragma once
#include "Programs/CompetitionDriver.h"
#include "pros/misc.h"

class CataDriver : public CompetitionDriver {

public:

    CataDriver(Robot& robot, DRIVE_TYPE driveType):
        CompetitionDriver(robot, driveType)
    {}
        
private:

    bool wasLimitSwitchOn = false;
    bool isFirstTick = true;

    void initDriver() override;
    void handleSecondaryActions() override;
};