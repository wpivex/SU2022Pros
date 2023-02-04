#pragma once

#include "main.h"
#include "Localizer.h"
#include "Subsystems/Drive/Drive.h"
#include "Algorithms/FixedRingQueue.h"

class Odometry : public Localizer {

private:
    pros::IMU imu;


    double currentX, currentY, currentHeading;
    double prevLeftDistance, prevRightDistance, prevHeading;

    bool imuValid = true;

    Drive& drive;

    RingQueue q;

    bool isOn = false;

public:

    Odometry(uint8_t imuPort, Drive& drivetrain):
        imu(imuPort),
        drive(drivetrain),
        q(5)
    {}

    double getX() override; // inches
    double getY() override; // inches
    double getHeading() override; // radians
    double getRotation() override;
    
    void updatePositionTask() override; // blocking task used to update (x, y, heading)
    void init() override; // init imu

    void setPosition(double x, double y) override;
    void setHeading(double headingRadians) override;
    void setRotation(double rotationRadians) override;
};