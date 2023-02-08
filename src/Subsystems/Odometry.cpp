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


    try {

        drive.resetDistance();
        prevLeftDistance = 0;
        prevRightDistance = 0;
        prevHeading = getRawHeading();

        double gpsX, gpsY, gpsHeading;
        double biasX = 0, biasY = 0, biasHeading = 0;

        const double K_POSITION = 0.03;
        const double K_HEADING = 0.01;

        pros::screen::set_pen(0x00FF0000);

        while (true) {

            pros::screen::erase();
            pros::lcd::clear();

            if (gps.get_error() > 0.015) {
                pros::screen::fill_rect(0, 0, 200, 200);
            }

            pros::lcd::print(0, "Filtered: %.2f %.2f %.2f",currentX, currentY, currentHeading);
            pros::lcd::print(1, "Odom/IMU: %.2f %.2f %.2f", odomX, odomY, getRawHeading());
            pros::lcd::print(2, "Individual IMU: %.2f %.2f", -getRadians(imuA.get_heading()), -getRadians(imuB.get_heading()));
            pros::lcd::print(3, "GPS: %.2f %.2f %.2f", gpsX, gpsY, gpsHeading);
            pros::lcd::print(4, "Bias: %.2f %.2f %.2f", biasX, biasY, biasHeading);

            pros::lcd::print(5, "GPS error: %f", gps.get_error());

            pros::delay(10);

            double left = drive.getLeftDistance();
            double right = drive.getRightDistance();
            double heading = getRawHeading();

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
            gpsX = status.x * METERS_TO_INCHES;
            gpsY = status.y * METERS_TO_INCHES;
            gpsHeading = fmod(-status.yaw + 360, 360) * M_PI / 180;

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
        

            pros::delay(10);
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
