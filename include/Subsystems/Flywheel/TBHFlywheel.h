#pragma once
#include "Flywheel.h"
#include "Algorithms/TBH.h"
#include "misc/MathUtility.h"
#include <vector>
#include "main.h"

// 3600 rpm 1:1 cart, but programmed as default 200rpm cart
class TBHFlywheel : public Flywheel {

private:
    TBH tbh;
    double ratio = 18;
    pros::MotorGroup motors;
    bool hasSetStopped = false;

public:

    TBHFlywheel(std::initializer_list<int8_t> flywheelMotors, double tbhConstant, std::vector<DataPoint> voltRpmData, double startSpeed = 0):
        motors(flywheelMotors),
        tbh(tbhConstant, startSpeed, voltRpmData)
    {}

    void setVelocity(double velocity) override;
    double getTargetVelocity() override;
    double getCurrentVelocity() override;
    void maintainVelocityTask() override;


};