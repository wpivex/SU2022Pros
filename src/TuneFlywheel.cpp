#include "main.h"
#include "TuneFlywheel.h"
#include "Algorithms/FixedRingQueue.h"

int volts = 12;

void tuneFlywheel(Robot& robot, ControllerSM& controller) {

    robot.flywheel->setRawVoltage(volts);
    pros::lcd::initialize();

    RingQueue q(50);

    while (true) {

        float speed = robot.flywheel->getCurrentVelocity();

        q.push(speed);

        pros::lcd::clear();
        pros::lcd::print(0, "Input Voltage (volts): %d", volts);
        pros::lcd::print(1, "Raw flywheel speed (rpm): %f", speed);
        pros::lcd::print(2, "Raw flywheel speed (rpm): %f", q.getAverage());

        if (controller.pressed(DIGITAL_UP)) {
            if (volts < 12) volts++;
            robot.flywheel->setRawVoltage(volts);
        } else if (controller.pressed(DIGITAL_DOWN)) {
            if (volts > 1) volts--;
            robot.flywheel->setRawVoltage(volts);
        }

        controller.updateButtonState();

        pros::delay(10);
    }
}