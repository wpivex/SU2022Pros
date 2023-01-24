#include "Subsystems/Flywheel/TBHFlywheel.h"

BBFFlywheel(std::initializer_list<int8_t> flywheelMotors, std::vector<DataPoint> voltRpmData, double startSpeed, double toleranceRPM):
    Flywheel(flywheelMotors, voltRpmData, startSpeed),
    tolerance(toleranceRPM)
{}

// Given the current velocity, return a goal velocity based on bbf algorithm to minimize error
float BBFFlywheel::getNextMotorVoltage(float currentRPM) {

    if (currentRPM < targetRPM - tolerance) {
        return 0;
    } else if (currentRPM < targetRPM + toleranace) {
        return rpmToVolt(targetRPM)
    } else {
        return 12;
    }
}