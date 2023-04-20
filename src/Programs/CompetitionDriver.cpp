#include "Programs/CompetitionDriver.h"
#include "misc/ProsUtility.h"
#include "pros/llemu.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"

void CompetitionDriver::runDriver() {
    pros::lcd::initialize();
    robot.drive->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    initDriver();

    // if (true && robot.localizer) {
    //     pros::Task taskOdometry([&] {
    //         robot.localizer->updatePositionTask();
    //     });
    // }

    while (true) {

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

void CompetitionDriver::handleDrivetrain() {

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