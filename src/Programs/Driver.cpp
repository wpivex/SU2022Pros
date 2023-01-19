#include "Programs/Driver.h"
#include "misc/ProsUtility.h"
#include "pros/llemu.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"

void Driver::runDriver() {
    pros::lcd::initialize();
    // Reinitialize flap position
    robot.shooterFlap->set_value(flapUp);
    robot.drive->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    while (true) {
        pros::lcd::clear();
        pros::lcd::print(0, "Target: %f", robot.flywheel->getTargetVelocity());
        pros::lcd::print(1, "Current: %f", robot.flywheel->getCurrentVelocity());
        pros::lcd::print(2, "Intake speed: %f", shootSpeed);

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

    if (controller.pressed(DIGITAL_Y)) {
        pros::Task taskFlywheel([&] {
            robot.flywheel->maintainVelocityTask();
        });
    }

    // Flywheel set speed
    robot.flywheel->setVelocity(speed);

}
