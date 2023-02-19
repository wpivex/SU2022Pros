#pragma once
#include "Programs/Driver.h"
#include "Algorithms/Alternator.h"
#include "pros/misc.h"
#include "Algorithms/Shooter.h"
#include "Algorithms/FixedRingQueue.h"

enum DRIVE_TYPE {TANK_DRIVE, ARCADE_DRIVE};

class CompetitionDriver : public Driver {

public:

    CompetitionDriver(Robot& robot, DRIVE_TYPE driveType, int defaultFlywheelSpeed):
        Driver(robot),
        drive(driveType),
        shootAlternator(3, 11, 7), // 70ms on / 70ms off
        DEFAULT_SPEED(defaultFlywheelSpeed),
        queue(100)
    {
        speed = DEFAULT_SPEED;
    }

    void runDriver() override;
        
private:

    const int DEFAULT_SPEED;

    void handleDrivetrain();
    void handleSecondaryActions();

    DRIVE_TYPE drive;

    Alternator shootAlternator;

    Shooter shooter;

    RingQueue queue;

    bool indexerOn;
    bool flapUp = true;
    int indexerTimer, indexerOffTimer;
    
    int speed;

    double shootSpeed = 1;

};