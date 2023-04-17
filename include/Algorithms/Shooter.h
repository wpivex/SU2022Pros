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
    int discNum = 0;
    /*
    State = 0: flywheel has not yet shot
    State = 1: flywheel has shot and speed has decreased
    */

public:

    int maxDisk = 3;

    void reset() {
        state = 0;
        discNum = 0;
    }

    double tickIntakeShootingSpeed(Robot& robot) {

        double diff = robot.flywheel->getTargetVelocity() - robot.flywheel->getCurrentVelocity();
        printf("%d, %f, %f, %f, %f, %f \n", state, diff, robot.flywheel->getTargetVelocity(), robot.flywheel->getCurrentVelocity(),
        robot.flywheel->motors.get_actual_velocities()[0], robot.flywheel->motors.get_actual_velocities()[1]);

        if (state == 0 && diff > 175) { // flywheel slowed down. Means we just finished shooting a disc
            discNum++;
            state = 1;
        } else if (state == 1 && diff < 0) { // we've reached back to full velocity, ready for the next shot
            state = 0;
        }

        //if (discNum == maxDisk) return 10;

        if (state == 1) { // if velocity is low after shooting disc, stop flywheel
            return 0;
        } 
        else {
            return (discNum == 0) ? FIRST_INTAKE_SPEED : AFTER_INTAKE_SPEED;
        }
    }
};