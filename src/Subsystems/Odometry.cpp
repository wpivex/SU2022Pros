#include "Subsystems/Localizer/Odometry.h"
#include "misc/MathUtility.h"


double Odometry::getX() { // inches

}

double Odometry::getY() { // inches

}

double Odometry::getHeading() { // radians
    return getRadians(imu.get_heading());
}
    
void Odometry::updatePositionTask() { // blocking task used to update (x, y, heading)

}

void Odometry::init() {
    imu.reset(true);
    pros::delay(1000);
}