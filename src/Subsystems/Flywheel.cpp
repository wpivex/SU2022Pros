#include "Subsystems/Flywheel/Flywheel.h"
#include "pros/llemu.hpp"


void Flywheel::setVelocity(double velocity) {
    targetRPM = velocity;
    if (velocity == 0) hasSetStopped = false;

    onSetpointUpdate();
}

double Flywheel::getTargetVelocity() {
    return targetRPM;
}

double Flywheel::getCurrentVelocity() {
    return average(motors.get_actual_velocities()) * ratio;
}

bool Flywheel::atTargetVelocity() {
    return fabs(getTargetVelocity() - getCurrentVelocity()) < 20;
}

void Flywheel::maintainVelocityTask() {
    
    while (true) {

        if (targetRPM == 0 && !hasSetStopped) {
            motors.brake();
            hasSetStopped = true;
        } else if (targetRPM != 0) {
            float currentRPM = getCurrentVelocity();
            //pros::lcd::print(0, "flywheel: %f", currentSpeed);
            float motorInputVolts = getNextMotorVoltage(currentRPM);
            motors.move_voltage(motorInputVolts * 1000); // millivolts
        }
        pros::delay(10);
    }
}

void Flywheel::setRawVoltage(double volts) {
    motors.move_voltage(volts * 1000);
}