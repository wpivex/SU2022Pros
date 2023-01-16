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
    State = 0: flywheel has not yet shot first disk
    State = 1: flywheel has shot first disk
    */

public:

    void reset() {
        state = 0;
    }

    double tickIntakeShootingSpeed(Robot& robot) {

        double diff = robot.flywheel->getTargetVelocity() - robot.flywheel->getCurrentVelocity();

        if (diff > 75) state = 1; // state == 1 means that the flyweel already shot the first disk

        if (diff > 50) { // if below target velocity by this number, stop flywheel
            return 0;
        } else {
            return (state == 1) ? AFTER_INTAKE_SPEED : FIRST_INTAKE_SPEED;
        }
    }
};