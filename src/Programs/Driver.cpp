#include "Programs/Driver.h"
#include "misc/ProsUtility.h"
#include "pros/rtos.hpp"

void Driver::runDriver() {
    pros::lcd::initialize();

    while (true) {
        pros::lcd::print(0, "%d", speed);

        // Handle drivetrain locomotion from joysticks (tank, arcade, etc.)
        handleDrivetrain();

        // Handle other things like intaking, shooting, etc.
        handleSecondaryActions();
        
        // Update button state machine for rising and falling edges
        controller.updateButtonState();

        // Enforce minimum polling cycle rate
        pros::delay(10);
    }
}

void Driver::handleDrivetrain() {

    float leftX = controller.getAxis(ANALOG_LEFT_X);
    float leftY = controller.getAxis(ANALOG_LEFT_Y);
    float rightX = controller.getAxis(ANALOG_RIGHT_X);
    float rightY = controller.getAxis(ANALOG_RIGHT_Y);

    if (drive == ARCADE_DRIVE) {
        float drive = leftY;
        float turn = rightX;
        float max = fmax(1.0, fmax(fabs(drive+turn), fabs(drive-turn)));

        float leftEffort = (drive + turn) / max;
        float rightEffort = (drive - turn) / max;
        robot.drive->setEffort(leftEffort, rightEffort);
    } else {
        robot.drive->setEffort(leftY, rightY);
    }

}

void Driver::handleSecondaryActions() {

    if (controller.pressed(DIGITAL_L1)) {
        if (speed < 3600) speed = fmin(speed + 200, 3600);
    }
    else if (controller.pressed(DIGITAL_L2)) {
        if (speed > 0) speed = fmax(speed - 200, 0);
    }
    else if (controller.pressed(DIGITAL_R2)) {
        robot.indexer->set_value(false);
        indexerOn = true;
        indexerTimer = pros::millis();
    }
    else if (controller.released(DIGITAL_R2)) {
        robot.indexer->set_value(true);
        indexerOn = false;
        indexerOffTimer = pros::millis();
    }

    if (indexerOn && pros::millis() - 250 > indexerTimer || (!indexerOn && pros::millis() - 300 < indexerOffTimer)) {
        setEffort(*robot.intake, -1);
    } else if (controller.pressing(DIGITAL_R1)) {
        setEffort(*robot.intake, 1);
    } else {
        robot.intake->brake();
    }

    robot.flywheel->setVelocity(speed);

}
