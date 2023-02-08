#pragma once

#include "main.h"
#include "IMULocalizer.h"
#include "Subsystems/Drive/Drive.h"
#include "Algorithms/FixedRingQueue.h"
#include "misc/MathUtility.h"

class Odometry : public IMULocalizer {

private:

    pros::GPS gps;


    double currentX, currentY, currentHeading;
    double odomX = 0, odomY = 0;
    double prevLeftDistance, prevRightDistance, prevHeading;

    bool isOn = false;

public:

    Odometry(Drive& drivetrain, uint8_t imuPortA, uint8_t imuPortB, uint8_t gpsPort, double gpsXOffset, double gpsYOffset):
        IMULocalizer(drivetrain, imuPortA, imuPortB),
        gps(gpsPort, gpsXOffset / METERS_TO_INCHES, gpsYOffset / METERS_TO_INCHES)
    {}

    double getX() override; // inches
    double getY() override; // inches
    double getHeading() override;
    
    void updatePositionTask() override; // blocking task used to update (x, y, heading)

    void setPosition(double x, double y) override;
};