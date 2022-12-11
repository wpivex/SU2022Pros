#include "Subsystems/Flywheel/TBHFlywheel.h"
#include "pros/llemu.hpp"


void TBHFlywheel::setVelocity(double velocity) {
    tbh.setTargetRPM(velocity);
    if (velocity == 0) hasSetStopped = false;
}

double TBHFlywheel::getTargetVelocity() {
    return tbh.getTargetRPM();
}

double TBHFlywheel::getCurrentVelocity() {
    return average(motors.get_actual_velocities()) * ratio;
}

bool TBHFlywheel::atTargetVelocity() {
    return fabs(getTargetVelocity() - getCurrentVelocity()) < 20;
}

void TBHFlywheel::maintainVelocityTask() {
    
    while (true) {

        if (tbh.getTargetRPM() == 0 && !hasSetStopped) {
            motors.brake();
            hasSetStopped = true;
        } else if (tbh.getTargetRPM() != 0) {
            float currentSpeed = getCurrentVelocity();
            pros::lcd::print(0, "flywheel: %f", currentSpeed);
            float motorInputVolts = tbh.getNextMotorVoltage(currentSpeed);
            motors.move_voltage(motorInputVolts * 1000); // millivolts
        }
        pros::delay(10);
    }
}