#include "Programs/CataDriver.h"
#include "misc/ProsUtility.h"
#include "pros/llemu.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"

void CataDriver::initDriver() {

}

void CataDriver::handleSecondaryActions() {

    // Cata. L2 spins forward, releasing stops
    if (controller.pressing(DIGITAL_L2)) {
        setEffort(*robot.cata, 1);
    } else {
        setEffort(*robot.cata, 0);
    }

    // Roller mech controls
    if (controller.pressing(DIGITAL_L1)) {
        setEffort(*robot.roller, -1);
    }
    else if (controller.pressing(DIGITAL_L2)) {
        setEffort(*robot.roller, 1);
    }
    else {
        setEffort(*robot.roller, 0);
    }

    // R1 to intake. R2 to outtake. If both off, turn off intake
    // Roller mech controls
    if (controller.pressing(DIGITAL_R1)) {
        setEffort(*robot.intake, 1);
    }
    else if (controller.pressing(DIGITAL_R2)) {
        setEffort(*robot.intake, -1);
    }
    else {
        setEffort(*robot.intake, 0);
    }

    // Endgame mech. Activate if B pressed
    if (controller.pressed(DIGITAL_B)) {
        robot.endgame->set_value(true);
    }

}
