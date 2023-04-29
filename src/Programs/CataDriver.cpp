#include "Programs/CataDriver.h"
#include "misc/ProsUtility.h"
#include "pros/llemu.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"

void CataDriver::initDriver() {

    timeAfterButtonPress = pros::millis();
    valve.set_value(true);
}

void CataDriver::handleSecondaryActions() {

    // When L1 pressed, cata runs until rising edge of limit switch
    bool isLimitSwitchOn = robot.limitSwitch->get_value();

    if (controller.pressed(DIGITAL_DOWN)) {
        valve.set_value(true);
    }
    if (controller.pressed(DIGITAL_UP)) {
        valve.set_value(false);
    }

    if (controller.pressed(DIGITAL_L1)) {
        canIntake = false;
        timeAfterButtonPress = pros::millis();
        setEffort(*robot.cata, 1);
    }

    if (isLimitSwitchOn && pros::millis() - timeAfterButtonPress > 800) {
        canIntake = true;
        setEffort(*robot.cata, 0);
    }
    
    wasLimitSwitchOn = isLimitSwitchOn;


    // Roller mech controls
    if (controller.pressing(DIGITAL_L2)) {
        setEffort(*robot.roller, -1);
    }
    else {
        setEffort(*robot.roller, 0);
    }

    // R1 to intake. R2 to outtake. If both off, turn off intake
    // Intake mech controls
    if (controller.pressing(DIGITAL_R2) && canIntake) {
        setEffort(*robot.intake, 1);
    }
    else if (controller.pressing(DIGITAL_R1) || !canIntake) {
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
