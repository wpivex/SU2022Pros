#include "Programs/FlywheelDriver.h"
#include "misc/ProsUtility.h"
#include "pros/llemu.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"

void FlywheelDriver::initDriver() {
    // Reinitialize flap position
    robot.shooterFlap->set_value(flapUp);

    if (true && robot.flywheel) {
        pros::Task taskFlywheel([&] {
            robot.flywheel->maintainVelocityTask();
        });
    }
}

void FlywheelDriver::handleSecondaryActions() {

    pros::lcd::clear();
    pros::lcd::print(0, "Flywheel target velocity: %.2f", robot.flywheel->getTargetVelocity());
    pros::lcd::print(1, "Flywheel actual velocity: %.2f", robot.flywheel->getCurrentVelocity());

    // Flywheel Speed Controls
    if (controller.pressed(DIGITAL_UP)) {
        if (speed < 3600) speed = fmin(speed + 100, 3600);
    }
    else if (controller.pressed(DIGITAL_DOWN)) {
        if (speed > 0) speed = fmax(speed - 100, 0);
    }
    if(controller.pressed(DIGITAL_RIGHT)) {
        // Set speed back to default speed
        speed = DEFAULT_SPEED;
    }

    // Roller mech controls
    if (controller.pressing(DIGITAL_L1)) {
        setEffort(*robot.roller, -1);
    }
    else if (controller.pressing(DIGITAL_L2)) {
        setEffort(*robot.roller, 1);
    }
    else{
        setEffort(*robot.roller, 0);
    }

    // Indexer controls
    if (controller.pressed(DIGITAL_R2)) {
        robot.indexer->set_value(false);
        indexerOn = true;
        indexerTimer = pros::millis();
    }
    else if (controller.released(DIGITAL_R2)) {
        robot.indexer->set_value(true);
        indexerOn = false;
        indexerOffTimer = pros::millis();
    } 
    else if (controller.pressed(DIGITAL_R1)) {
        //shooter.reset();
    }

    // Flywheel flap toggle
    if (controller.pressed(DIGITAL_X)) {
        flapUp = !flapUp;
        robot.shooterFlap->set_value(flapUp);
    }

    // Running the indexer via the intake
    if (indexerOn && pros::millis() - 250 > indexerTimer || (!indexerOn && pros::millis() - 300 < indexerOffTimer)) {
        setEffort(*robot.intake, 1);
    } else if (controller.pressing(DIGITAL_R1)) {
        setEffort(*robot.intake, -1);
    } else {
        robot.intake->brake();
    }

    // Endgame mech. Activate if B pressed
    if (controller.pressed(DIGITAL_B)) {
        robot.endgame->set_value(true);
    }

    // Flywheel set speed
    robot.flywheel->setVelocity(speed);

}
