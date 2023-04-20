#pragma once
#include "Programs/CompetitionDriver.h"
#include "pros/misc.h"

class FlywheelDriver : public CompetitionDriver {

public:

    FlywheelDriver(Robot& robot, DRIVE_TYPE driveType, int defaultFlywheelSpeed):
        CompetitionDriver(robot, driveType),
        DEFAULT_SPEED(defaultFlywheelSpeed)
    {
        speed = DEFAULT_SPEED;
    }
        
private:

    const int DEFAULT_SPEED;

    void initDriver() override;
    void handleSecondaryActions() override;

    bool indexerOn;
    bool flapUp = true;
    int indexerTimer, indexerOffTimer;
    
    int speed;

    double shootSpeed = 1;

};