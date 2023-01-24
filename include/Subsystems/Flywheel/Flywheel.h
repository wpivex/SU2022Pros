#pragma once
#include "Flywheel.h"
#include "Algorithms/TBH.h"
#include "misc/MathUtility.h"
#include <vector>
#include "main.h"

// 3600 rpm 1:1 cart, but programmed as default 200rpm cart
class TBHFlywheel : public Flywheel {

protected:

    std::vector<DataPoint> data;

    double targetRPM;
    double ratio = 18;
    pros::MotorGroup motors;
    bool hasSetStopped = false;

    virtual void onSetpointUpdate() {}

public:

    TBHFlywheel(std::initializer_list<int8_t> flywheelMotors, std::vector<DataPoint> voltRpmData, double startSpeed):
        motors(flywheelMotors),
        data(voltRpmData),
        targetRPM(startSpeed)
    {}

    void setVelocity(double velocity);
    double getTargetVelocity();
    double getCurrentVelocity();
    void maintainVelocityTask();
    
    bool atTargetVelocity();

    void setRawVoltage(double volts);

    virtual double getNextMotorVoltage(double currentRPM) = 0;


};