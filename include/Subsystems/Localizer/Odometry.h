#pragma once

#include "main.h"
#include "Localizer.h"

class Odometry : public Localizer {

private:
    pros::IMU imu;
    pros::ADIEncoder left, right, back;
    double diameter, backDistance;

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
    
    void updatePositionTask() override; // blocking task used to update (x, y, heading)
    void init() override; // init imu

};