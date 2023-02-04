#pragma once

#include "main.h"
#include "misc/MathUtility.h"
#include "pros/motors.h"

class Drive {

private:

    pros::MotorGroup leftMotors, rightMotors;
    const double MOTOR_ROT_TO_LINEAR_INCHES;

    double leftPositionAtZero;
    double rightPositionAtZero;

    double _getMotorDistance(pros::MotorGroup& motors);
    double _getMotorCurrent(pros::MotorGroup& motors);

public:

    const double TRACK_WIDTH;

    Drive(std::initializer_list<int8_t> left, std::initializer_list<int8_t> right,
    pros::motor_gearset_e_t internalGearRatio, double externalGearRatio, double wheelDiameterInches,
    double trackWidthInches);

    // bounded -1 to 1
    void setEffort(double left, double right);

    // linear velocity in inch/sec
    void setVelocity(double left, double right);

    void stop();

    void setBrakeMode(pros::motor_brake_mode_e_t mode);

    // reset distance travelled by the left and right motors
    void resetDistance();

    // Get distance travelled by the left wheels in linear inches
    double getLeftDistance();

    // Get distance travelled by the right wheels in linear inches
    double getRightDistance();

    // Get average distance travelled by the left and right wheels in linear inches
    double getDistance();

    // get motor current for motors in amps
    double getCurrent();

};