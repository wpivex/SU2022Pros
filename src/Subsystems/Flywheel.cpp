#include "Subsystems/Flywheel/Flywheel.h"
#include "pros/llemu.hpp"


void Flywheel::setVelocity(double velocity) {
    targetRPM = velocity;
    if (velocity == 0) hasSetStopped = false;

    isFirstCrossover = true;
}

double Flywheel::getTargetVelocity() {
    return targetRPM;
}

double Flywheel::getCurrentVelocity() {
    return motors.get_actual_velocities()[0] * ratio;
}

bool Flywheel::atTargetVelocity() {
    return fabs(getTargetVelocity() - getCurrentVelocity()) < 20;
}

void Flywheel::maintainVelocityTask() {

    if (isOn) return;
    isOn = true;
    
    while (true) {

        if (targetRPM == 0 && !hasSetStopped) {
            motors.brake();
            hasSetStopped = true;
        } else if (targetRPM != 0) {
            float currentRPM = getCurrentVelocity();
            //pros::lcd::print(0, "flywheel: %f", getCurrentVelocity());
            targetVoltage = getNextMotorVoltage(currentRPM);
            motors.move_voltage(targetVoltage * 1000); // millivolts
        }
        pros::delay(10);
    }
}

void Flywheel::setRawVoltage(double volts) {
    targetVoltage = volts;
    motors.move_voltage(volts * 1000);
}

double Flywheel::getTargetVoltage() {
    return targetVoltage;
}