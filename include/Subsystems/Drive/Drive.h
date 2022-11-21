#pragma once

#include "main.h"
#include "misc/MathUtility.h"
#include "pros/motors.h"

class Drive {

private:

    pros::MotorGroup leftMotors, rightMotors;
    const double MOTOR_ROT_TO_LINEAR_INCHES;

public:

    const double TRACK_WIDTH;

    Drive(std::initializer_list<int8_t> left, std::initializer_list<int8_t> right,
    pros::motor_gearset_e_t internalGearRatio, double externalGearRatio, double wheelDiameterInches,
    double trackWidthInches):
        leftMotors(left),
        rightMotors(right),
        TRACK_WIDTH(trackWidthInches),
        MOTOR_ROT_TO_LINEAR_INCHES(externalGearRatio * M_PI * wheelDiameterInches)
    {
        leftMotors.set_gearing(internalGearRatio);
        rightMotors.set_gearing(internalGearRatio);
        leftMotors.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
        rightMotors.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
    }

    // bounded -1 to 1
    void setEffort(double left, double right) {
        leftMotors.move_voltage(left * 12000); // take in millivolts
        rightMotors.move_voltage(right * 12000);
    }

    // linear velocity in inch/sec
    void setVelocity(double left, double right) {
        leftMotors.move_velocity(left / MOTOR_ROT_TO_LINEAR_INCHES * 60);
        rightMotors.move_velocity(right / MOTOR_ROT_TO_LINEAR_INCHES * 60);
    }

    void setBrakeMode(pros::motor_brake_mode_e_t mode) {
        leftMotors.set_brake_modes(mode);
        rightMotors.set_brake_modes(mode);
    }

    // reset distance travelled by the left and right motors
    void resetDistance() {
        leftMotors.tare_position();
        rightMotors.tare_position();
    }

    // Get distance travelled by the left wheels in linear inches
    double getLeftDistance() {
        double rotations = average(leftMotors.get_positions());
        return rotations * MOTOR_ROT_TO_LINEAR_INCHES;
    }

    // Get distance travelled by the right wheels in linear inches
    double getRightDistance() {
        double rotations = average(rightMotors.get_positions());
        return rotations * MOTOR_ROT_TO_LINEAR_INCHES;
    }
};