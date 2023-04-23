#pragma once

#include "main.h"
#include "Localizer.h"
#include "Subsystems/Drive/Drive.h"
#include "Algorithms/FixedRingQueue.h"

class IMULocalizer : public Localizer {

protected:

    pros::IMU imuA;
    pros::IMU imuB;

    bool imuValidA = true;
    bool imuValidB = true;

    Drive& drive;

    RingQueue qA, qB;

    double getRawHeading();

public:

    IMULocalizer(Drive& drivetrain, uint8_t imuPortA, uint8_t imuPortB):
        drive(drivetrain),
        imuA(imuPortA),
        imuB(imuPortB),
        qA(5),
        qB(5)
    {}

    virtual double getHeading() override; // radians
    
    virtual void updatePositionTask() override; // blocking task used to update (x, y, heading)
    virtual void init() override; // init imu

    virtual void setPosition(double x, double y) override;
    virtual void setHeading(double headingRadians) override;
};