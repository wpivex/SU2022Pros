#pragma once

#include "main.h"
#include "Localizer.h"

class Odometry : public Localizer {

private:
    pros::IMU imu;
    pros::ADIEncoder left, right, back;
    double diameter, backDistance;

    double currentX, currentY, currentHeading;

public:

    Odometry(uint8_t imuPort, pros::ADIEncoder leftEncoder, pros::ADIEncoder rightEncoder, pros::ADIEncoder backEncoder,
    double wheelDiameter, double backEncoderDistance):
        imu(imuPort),
        left(leftEncoder),
        right(rightEncoder),
        back(backEncoder),
        diameter(wheelDiameter),
        backDistance(backEncoderDistance)
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