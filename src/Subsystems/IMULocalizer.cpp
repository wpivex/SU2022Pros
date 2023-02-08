#include "Subsystems/Localizer/IMULocalizer.h"
#include "misc/MathUtility.h"
#include <stdexcept>


double IMULocalizer::getX() { // inches
    return 0;
}

double IMULocalizer::getY() { // inches
    return 0;
}

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
        imuValidA = false;
    }

    if (!imuValidA && !imuValidB) throw std::runtime_error("Both IMU disconnect.");
    else if (!imuValidA) return headingB;
    else if (!imuValidB) return headingA;

    return headingA + deltaInHeading(headingB, headingA) / 2.0;
}
    
void IMULocalizer::updatePositionTask() { // blocking task used to update (x, y, heading)
    // nothing
}

void IMULocalizer::init() {
    pros::delay(500);
    imuA.reset(false);
    imuB.reset(true);
    pros::delay(1000);
    while (imuA.get_heading() == POS_INF || imuB.get_heading() == POS_INF) pros::delay(10);

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