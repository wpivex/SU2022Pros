#include "Subsystems/Drive/Drive.h"


Drive::Drive(std::initializer_list<int8_t> left, std::initializer_list<int8_t> right, pros::motor_gearset_e_t internalGearRatio, double externalGearRatio, double wheelDiameterInches, double trackWidthInches):
    leftMotors(left),
    rightMotors(right),
    TRACK_WIDTH(trackWidthInches),
    MOTOR_ROT_TO_LINEAR_INCHES(externalGearRatio * M_PI * wheelDiameterInches)
{
    leftMotors.set_gearing(internalGearRatio);
    rightMotors.set_gearing(internalGearRatio);
    leftMotors.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
    rightMotors.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);

    resetDistance();
}

// bounded -1 to 1
void Drive::setEffort(double left, double right) {
    pros::lcd::clear();
    pros::lcd::print(0, "%.2f %.2f", left, right);
    leftMotors.move_voltage(left * 12000); // take in millivolts
    rightMotors.move_voltage(right * 12000);
}

// linear velocity in inch/sec
void Drive::setVelocity(double left, double right) {
    leftMotors.move_velocity(left / MOTOR_ROT_TO_LINEAR_INCHES * 60);
    rightMotors.move_velocity(right / MOTOR_ROT_TO_LINEAR_INCHES * 60);
}

void Drive::stop() {
    leftMotors.brake();
    rightMotors.brake();
}

void Drive::setBrakeMode(pros::motor_brake_mode_e_t mode) {
    leftMotors.set_brake_modes(mode);
    rightMotors.set_brake_modes(mode);
}

// reset distance travelled by the left and right motors
void Drive::resetDistance() {
    leftPositionAtZero = _getMotorDistance(leftMotors);
    rightPositionAtZero = _getMotorDistance(rightMotors);
}

double Drive::_getMotorDistance(pros::MotorGroup& motors) {

    double sum = 0;
    int size = 0;
    for (int i = 0; i < motors.size(); i++) {
        double pos = motors[i].get_position();
        if (pos != PROS_ERR_F) {
            size++;
            sum += pos;
        }
    }

    if (size == 0) return 0;

    return sum / size * MOTOR_ROT_TO_LINEAR_INCHES;
}

// Get distance travelled by the left wheels in linear inches
double Drive::getLeftDistance() { return _getMotorDistance(leftMotors) - leftPositionAtZero; }

// Get distance travelled by the right wheels in linear inches
double Drive::getRightDistance() { return _getMotorDistance(rightMotors) - rightPositionAtZero; }

// Get average distance travelled by the left and right wheels in linear inches
double Drive::getDistance() {
    return (getLeftDistance() + getRightDistance()) / 2.0;
}

// get motor current for motor group in amps
double Drive::_getMotorCurrent(pros::MotorGroup& motors) {

    double sum = 0;
    int size = 0;
    for (int i = 0; i < motors.size(); i++) {
        double current = motors[i].get_current_draw(); // returns in mA
        if (current != PROS_ERR_F) {
            size++;
            sum += current;
        }
    }

    if (size == 0) return 0;

    return (sum / size) / 1000.0; // convert to amps
}

double Drive::getCurrent() {
    double left = _getMotorCurrent(leftMotors);
    double right = _getMotorCurrent(rightMotors);
    return (left + right) / 2.0;
}