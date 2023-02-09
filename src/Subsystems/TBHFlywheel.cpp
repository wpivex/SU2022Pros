#include "Subsystems/Flywheel/TBHFlywheel.h"

TBHFlywheel::TBHFlywheel(std::initializer_list<int8_t> flywheelMotors, std::vector<DataPoint> voltRpmData, std::vector<DataPoint> rpmDistanceFlapDownData, std::vector<DataPoint> rpmDistanceFlapUpData, double startSpeed, double gainConstant):
    Flywheel(flywheelMotors, voltRpmData, rpmDistanceFlapDownData, rpmDistanceFlapUpData, startSpeed),
    gain(gainConstant)
{}

// Given the current velocity, return a goal velocity based on tbh algorithm to minimize error
double TBHFlywheel::getNextMotorVoltage(double currentRPM) {
    
    float error = targetRPM - currentRPM; // calculate the error;
    output += gain * error; // integrate the output

    output = clamp(output, 0, 12); // bound output to possible voltages
        
    if (sign(error) != sign(prevError)) { // if zero crossing,

        if (isFirstCrossover) { // First zero crossing after a new set velocity command
            // Set drive to the open loop approximation
            output = rpmToVolt(data, targetRPM);
        } else {
            output = 0.5 * (output + tbh); // Take Back Half
            isFirstCrossover = false;
        }

        tbh = output;// update Take Back Half variable
        prevError = error; // save the previous error
    }

    return output;

}