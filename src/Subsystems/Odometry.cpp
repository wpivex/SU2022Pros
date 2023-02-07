#include "Subsystems/Localizer/Odometry.h"
#include "misc/MathUtility.h"
#include <stdexcept>


double Odometry::getX() { // inches
    return currentX;
}

double Odometry::getY() { // inches
    return currentY;
}

double Odometry::getHeading() {

    if (!imuValidA && !imuValidB) throw std::runtime_error("Both IMU disconnect.");
    return currentHeading;

}

double Odometry::getRawHeading() { // radians


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
    
void Odometry::updatePositionTask() { // blocking task used to update (x, y, heading)

    if (isOn) return;
    isOn = true;

    try {

        drive.resetDistance();
        prevLeftDistance = 0;
        prevRightDistance = 0;
        prevHeading = getRawHeading();

        double gpsX, gpsY, gpsHeading;
        double biasX = 0, biasY = 0, biasHeading = 0;

        const double K_POSITION = 0.05;
        const double K_HEADING = 0.01;

        while (true) {

            pros::delay(10);

            double left = drive.getLeftDistance();
            double right = drive.getRightDistance();
            double heading = getHeading();

            double deltaLeft = left - prevLeftDistance;
            double deltaRight = right - prevRightDistance;
            double deltaHeading = heading - prevHeading;

            double arcLength = (deltaLeft + deltaRight) / 2;
            if (deltaHeading == 0) {
                // rare case where robot moved perfectly straight this tick
                currentX += arcLength * cos(heading);
                currentY += arcLength * sin(heading);

            } else {
                // The robot moved in an arc with radius = arcLength / deltaTheta
                double radius = arcLength / deltaHeading;
                currentX += radius * (cos(heading) - cos(prevHeading));
                currentY += radius * (sin(heading) - sin(prevHeading));
            }
            
            prevLeftDistance = left;
            prevRightDistance = right;
            prevHeading = heading;

            // Calculate gps
            pros::c::gps_status_s_t status = gps.get_status();
            gpsX = status.x;
            gpsY = status.y;
            gpsHeading = status.yaw;

            // Find filtered position
            currentX = odomX + biasX;
            currentY = odomY + biasY;
            currentHeading = heading + biasHeading;

            // Update bias from gps
            if (gps.get_error() < 0.015) { // we found that, below this value, gps reads stable values
                biasX += (gpsX - currentX) * K_POSITION;
                biasY += (gpsY - currentY) * K_POSITION;
                biasHeading += deltaInHeading(gpsHeading, currentHeading) * K_HEADING;
            }
            

            pros::lcd::clear();
            pros::lcd::print(0, "X: %f", currentX);
            pros::lcd::print(1, "Y: %f", currentY);
            pros::lcd::print(2, "Heading: %f", currentHeading * 180 / 3.1415);
        }
    } catch (std::runtime_error &e) {
        // nothing, stopping motors handled in main thread
    }


}

void Odometry::init() {
    pros::delay(500);
    imuA.reset(false);
    imuB.reset(true);
    pros::delay(1000);
    while (imuA.get_heading() == POS_INF || imuB.get_heading() == POS_INF) pros::delay(10);

}

void Odometry::setPosition(double x, double y) {
    currentX = x;
    odomX = x;
    currentY = y;
    odomY = y;
    
}

void Odometry::setHeading(double headingRadians) {
    double d = getDegrees(-headingRadians);
    d = fmod(fmod(d, 360) + 360, 360);
    imuA.set_heading(d);
    imuB.set_heading(d);
}