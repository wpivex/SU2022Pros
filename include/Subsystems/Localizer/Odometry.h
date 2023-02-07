#pragma once

#include "main.h"
#include "Localizer.h"
#include "Subsystems/Drive/Drive.h"
#include "Algorithms/FixedRingQueue.h"

class Odometry : public Localizer {

private:
    pros::IMU imuA;
    pros::IMU imuB;

    pros::GPS gps;


    double currentX, currentY, currentHeading;
    double odomX, odomY;
    double prevLeftDistance, prevRightDistance, prevHeading;

    bool imuValidA = true;
    bool imuValidB = true;

    Drive& drive;

    RingQueue qA, qB;

    bool isOn = false;

    double getRawHeading();

public:

    Odometry(uint8_t imuPortA, uint8_t imuPortB, uint8_t gpsPort, Drive& drivetrain):
        imuA(imuPortA),
        imuB(imuPortB),
        gps(gpsPort),
        drive(drivetrain),
        qA(5),
        qB(5)
    {}

    double getX() override; // inches
    double getY() override; // inches
    double getHeading() override; // radians
    
    void updatePositionTask() override; // blocking task used to update (x, y, heading)
    void init() override; // init imu

    void setPosition(double x, double y) override;
    void setHeading(double headingRadians) override;
};