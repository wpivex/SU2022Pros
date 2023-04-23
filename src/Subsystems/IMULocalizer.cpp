#include "Subsystems/Localizer/IMULocalizer.h"
#include "misc/MathUtility.h"
#include <stdexcept>


double IMULocalizer::getHeading() {

    return getRawHeading();

}

double IMULocalizer::getRawHeading() { // radians


    double headingA = -getRadians(imuA.get_heading());
    qA.push(headingA);

    double headingB = -getRadians(imuB.get_heading());
    qB.push(headingB);

    
    if (qA.isAllEqual()) {
        imuValidA = false;
    }
    if (qB.isAllEqual()) {
        imuValidB = false;
    }

    if (!imuValidA && !imuValidB) throw std::runtime_error("Both IMU disconnect.");
    else if (!imuValidA) return headingB;
    else if (!imuValidB) return headingA;

    double avg = headingA + deltaInHeading(headingB, headingA) / 2.0;
    return fmod(fmod(avg, 2*M_PI) + 2*M_PI, 2*M_PI);
}
    
void IMULocalizer::updatePositionTask() { // blocking task used to update (x, y, heading)
    // nothing
}

void IMULocalizer::init() {
    pros::lcd::print(0, "Initialization start.");
    pros::delay(500);
    imuA.reset(false);
    imuB.reset(true);
    pros::delay(1000);
    uint32_t endTime = pros::millis() + 5000;
    while (imuA.get_heading() == POS_INF && imuB.get_heading() == POS_INF && pros::millis() < endTime) pros::delay(10);
    pros::delay(1000);

    if (imuA.get_heading() == POS_INF) {
        imuValidA = false;
        pros::lcd::print(1, "IMU A disconnected. Still operational if IMU B is connected.");
    }
    if (imuB.get_heading() == POS_INF) {
        imuValidB = false;
        pros::lcd::print(1, "IMU B disconnected. Still operational if IMU A is connected.");
    }

    pros::lcd::print(0, "Initialization complete.");

}

void IMULocalizer::setPosition(double x, double y) {
    // nothing
}

void IMULocalizer::setHeading(double headingRadians) {
    double d = getDegrees(-headingRadians);
    d = fmod(fmod(d, 360) + 360, 360);
    imuA.set_heading(d);
    imuB.set_heading(d);
}