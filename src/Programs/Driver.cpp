#include "Programs/Driver.h"

void Driver::runDriver() {

    while (true) {

        // Handle drivetrain locomotion from joysticks (tank, arcade, etc.)
        handleDrivetrain();

        // Handle other things like intaking, shooting, etc.
        handleSecondaryActions();
        
        // Update button state machine for rising and falling edges
        buttonSM.updateButtonState();

        // Enforce minimum polling cycle rate
        pros::delay(10);
    }
}

void Driver::handleSecondaryActions() {

    const float INCREMENT = 200;

    // update flywheel speed with L1 and L2
    if (buttonSM.pressed(DIGITAL_L1)) {
        double target = fmin(4000, robot.flywheel->getVelocity() + INCREMENT);
        robot.flywheel->setVelocity(target);
    } else if (buttonSM.pressed(DIGITAL_L2)) {
        double target = fmax(0, robot.flywheel->getVelocity() - INCREMENT);
        robot.flywheel->setVelocity(target);
    }


    // Indexer is set to R2 state. R2: Intake forward with 0.25s delay, R1: intake reverse
    if (buttonSM.pressed(DIGITAL_R2)) {
        robot.indexer.set_value(true);
        indexerOn = true;
        indexerTimer = pros::millis();
    } else if (buttonSM.released(DIGITAL_R2)) {
        robot.indexer.set_value(false);
        indexerOn = false;
    }

    // Actually set the intake velocity
    if (indexerOn && pros::millis() - 250 > indexerTimer) {
        robot.intake.move_voltage(-12000);
    } else if (buttonSM.pressing(DIGITAL_R1)) {
        robot.intake.move_voltage(12000);
    } else {
        robot.intake.brake();
    }
}
