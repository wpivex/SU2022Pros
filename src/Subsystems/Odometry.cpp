#include "Subsystems/Localizer/Odometry.h"
#include "misc/MathUtility.h"
#include <stdexcept>


double Odometry::getX() { // inches
    return currentX;
}

double Odometry::getY() { // inches
    return currentY;
}

double Odometry::getHeading() { // radians


    double deg = imu.get_heading();
    q.push(deg);

    if (!imuValid || q.isAllEqual()) {
        imuValid = false;
        throw std::runtime_error("IMU disconnect");
    }

    return -getRadians(deg);
}
    
void Odometry::updatePositionTask() { // blocking task used to update (x, y, heading)

    if (isOn) return;
    isOn = true;

    try {

        drive.resetDistance();
        prevLeftDistance = 0;
        prevRightDistance = 0;
        prevHeading = getHeading();

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
            

            pros::lcd::clear();
            pros::lcd::print(0, "X: %f", currentX);
            pros::lcd::print(1, "Y: %f", currentY);
            pros::lcd::print(2, "Heading: %f", heading * 180 / 3.1415);
        }
    } catch (std::runtime_error &e) {
        // nothing, stopping motors handled in main thread
    }


}

void Odometry::init() {
    pros::delay(500);
    imu.reset(true);
    pros::delay(1000);
    while (imu.get_heading() == POS_INF) pros::delay(10);

}

void Odometry::setPosition(double x, double y) {
    currentX = x;
    currentY = y;
    
}

void Odometry::setHeading(double headingRadians) {
    double d = getDegrees(-headingRadians);
    d = fmod(fmod(d, 360) + 360, 360);
    imu.set_heading(d);
}

double Odometry::getRotation() {
    return -getRadians(imu.get_rotation());
}

void Odometry::setRotation(double rotationRadians) {
    imu.set_rotation(getDegrees(-rotationRadians));
}