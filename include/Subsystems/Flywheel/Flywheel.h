#pragma once
#include "misc/MathUtility.h"
#include <vector>
#include "Algorithms/ConversionData.h"
#include "main.h"

// 3600 rpm 1:1 cart, but programmed as default 200rpm cart
class Flywheel {

protected:

    std::vector<DataPoint> data;

    double targetRPM;
    double ratio = 36;
    pros::MotorGroup motors;
    bool hasSetStopped = false;

    double targetVoltage;

    bool isFirstCrossover = true;

    bool isOn = false;

public:

    Flywheel(std::initializer_list<int8_t> flywheelMotors, std::vector<DataPoint> voltRpmData, double startSpeed):
        motors(flywheelMotors),
        data(voltRpmData),
        targetRPM(startSpeed)
    {
        motors.set_gearing(pros::E_MOTOR_GEAR_100);
    }

    void setVelocity(double velocity);
    double getTargetVelocity();
    double getCurrentVelocity();
    void maintainVelocityTask();
    double getTargetVoltage();
    
    bool atTargetVelocity();

    void setRawVoltage(double volts);

    virtual double getNextMotorVoltage(double currentRPM) = 0;


};