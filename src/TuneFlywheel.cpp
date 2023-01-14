#include "main.h"
#include "TuneFlywheel.h"

int volts = 12;

void tuneFlywheel(Robot& robot, ControllerSM& controller) {

    robot.flywheel->setRawVoltage(volts);
    pros::lcd::initialize();

    while (true) {

        pros::lcd::clear();
        pros::lcd::print(0, "Input Voltage (volts): %f", volts);
        pros::lcd::print(1, "Flywheel speed (rpm): %f", robot.flywheel->getCurrentVelocity());

        if (controller.pressed(DIGITAL_UP)) {
            if (volts < 12) volts++;
            robot.flywheel->setRawVoltage(volts);
        } else if (controller.pressed(DIGITAL_DOWN)) {
            if (volts > 1) volts--;
            robot.flywheel->setRawVoltage(volts);
        }

        pros::delay(10);
    }
}