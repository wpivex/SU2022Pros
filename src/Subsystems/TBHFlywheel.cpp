#include "Subsystems/Flywheel/TBHFlywheel.h"


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
    return fabs(getTargetVelocity() - getCurrentVelocity()) < 40;
}

void TBHFlywheel::maintainVelocityTask() {
    
    while (true) {

        if (tbh.getTargetRPM() == 0 && !hasSetStopped) {
            motors.brake();
            hasSetStopped = true;
        } else if (tbh.getTargetRPM() != 0) {
            float currentSpeed = getCurrentVelocity();
            float motorInputVolts = tbh.getNextMotorVoltage(currentSpeed);
            motors.move_voltage(motorInputVolts * 1000); // millivolts
        }
        pros::delay(10);
    }
}