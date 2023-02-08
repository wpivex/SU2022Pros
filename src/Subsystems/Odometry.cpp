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
    
void Odometry::updatePositionTask() { // blocking task used to update (x, y, heading)

    if (isOn) return;
    isOn = true;

    // Initialize odom position
    pros::c::gps_status_s_t status = gps.get_status();
    odomX = status.x;
    odomY = status.y;

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

                
            pros::lcd::print(0, "X pos: %f",currentX);
            pros::lcd::print(1, "Y pos: %f", currentY);
            pros::lcd::print(2, "Heading: %f", currentHeading);

            pros::lcd::print(3, "X odom: %f",odomX);
            pros::lcd::print(4, "Y odom: %f", odomY);

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
                odomX += arcLength * cos(heading);
                odomY += arcLength * sin(heading);

            } else {
                // The robot moved in an arc with radius = arcLength / deltaTheta
                double radius = arcLength / deltaHeading;
                odomX += radius * (cos(heading) - cos(prevHeading));
                odomY += radius * (sin(heading) - sin(prevHeading));
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


void Odometry::setPosition(double x, double y) {
    currentX = x;
    odomX = x;
    currentY = y;
    odomY = y;
}
