#pragma once
#include "Programs/Driver.h"
#include "Algorithms/Alternator.h"
#include "pros/misc.h"
#include "Algorithms/Shooter.h"
#include "Algorithms/FixedRingQueue.h"

enum DRIVE_TYPE {TANK_DRIVE, ARCADE_DRIVE};

class CompetitionDriver : public Driver {

public:

    CompetitionDriver(Robot& robot, DRIVE_TYPE driveType):
        Driver(robot),
        drive(driveType)
    {}

    void runDriver() override;
        
private:

    void handleDrivetrain();
    virtual void initDriver() {}
    virtual void handleSecondaryActions() {}

    DRIVE_TYPE drive;

};