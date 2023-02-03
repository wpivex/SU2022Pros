#include "Subsystems/Flywheel/BBFFlywheel.h"

BBFFlywheel::BBFFlywheel(std::initializer_list<int8_t> flywheelMotors, std::vector<DataPoint> voltRpmData, double startSpeed, double toleranceRPM):
    Flywheel(flywheelMotors, voltRpmData, startSpeed),
    tolerance(toleranceRPM)
{}

// Given the current velocity, return a goal velocity based on bbf algorithm to minimize error
double BBFFlywheel::getNextMotorVoltage(double currentRPM) {

    if (currentRPM < targetRPM - tolerance) {
        return 12;
    } else if (currentRPM < targetRPM + tolerance) {
        return rpmToVolt(data, targetRPM);
    } else {
        return 0;
    }
}