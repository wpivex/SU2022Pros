#pragma once
#include "main.h"
#include "Subsystems/Robot.h"
#include "misc/ProsUtility.h"

// First shot intake speed = -1, otherwise intake speed = -0.5
class Shooter {

private:

    const double FIRST_INTAKE_SPEED = -1;
    const double AFTER_INTAKE_SPEED = -1;

    int state = 0;
    /*
    State = 0: flywheel getting to top speed
    State = 1: flywheel reached top speed
    State = 2: flywheel dropped from shooting
    */

public:

    void reset() {
        state = 0;
    }

    double tickIntakeShootingSpeed(Robot& robot) {

        double diff = robot.flywheel->getTargetVelocity() - robot.flywheel->getCurrentVelocity();

        if (state == 0 && diff < 50) state = 1;
        if (state == 1 && diff > 75) state = 2;

        if (diff > 75) { // flywheel speed drop
            return 0;
        } else {
            return (state == 2) ? AFTER_INTAKE_SPEED : FIRST_INTAKE_SPEED;
        }
    }
};