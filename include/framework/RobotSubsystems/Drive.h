#include "main.h"
#include "misc/MathUtility.h"

class Drive {

private:

    pros::MotorGroup leftMotors, rightMotors;
    double inchSecToMotorRpm;

public:

    Drive(std::initializer_list<pros::Motor> left, std::initializer_list<pros::Motor> right,
    pros::motor_gearset_e_t internalGearRatio, double externalGearRatio, double wheelDiameterInches):
        leftMotors(left),
        rightMotors(right)
    {
        leftMotors.set_gearing(internalGearRatio);
        rightMotors.set_gearing(internalGearRatio);

        inchSecToMotorRpm = 60.0 / (wheelDiameterInches * M_PI) / externalGearRatio;
    }

    // bounded -1 to 1
    void setEffort(double left, double right) {
        leftMotors.move_voltage(left * 12000);
        rightMotors.move_voltage(right * 12000);
    }

    // linear velocity in inch/sec
    void setVelocity(double left, double right) {
        leftMotors.move_velocity(left * inchSecToMotorRpm);
        rightMotors.move_velocity(right * inchSecToMotorRpm);
    }

    void setBrakeMode(pros::motor_brake_mode_e_t mode) {
        leftMotors.set_brake_modes(mode);
        rightMotors.set_brake_modes(mode);
    }

    void resetPosition() {
        leftMotors.tare_position();
        rightMotors.tare_position();
    }
    double getLeftPosition() {
        return average(leftMotors.get_positions());
    }
    double getRightPosition() {
        return average(rightMotors.get_positions());
    }
};