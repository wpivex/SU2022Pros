

#include "AutonomousFunctions/DriveFunctions.h"
#include "misc/MathUtility.h"
#include "pros/rtos.hpp"

// Check if targetHeading was set to default, in which case maintain the current heading and update targetHeading value
inline void setHeading(Robot& robot, double& targetHeading) {
    if (targetHeading == MAINTAIN_CURRENT_HEADING) targetHeading = robot.localizer->getHeading();
}

// Go forwards for some time while maintaining heading
void goForwardTimedU(Robot& robot, SimplePID&& pidHeading, double timeSeconds, double targetEffort, double targetHeading) {
    
    setHeading(robot, targetHeading);

    uint32_t endTime = pros::millis() + timeSeconds * 1000;

    while (pros::millis() < endTime) {

        double headingError = deltaInHeading(targetHeading, robot.localizer->getHeading());
        double deltaVelocity = pidHeading.tick(headingError);
        
        double left = targetEffort - deltaVelocity;
        double right = targetEffort + deltaVelocity;
        robot.drive->setEffort(left, right);

        pros::delay(10);
    }

    robot.drive->stop();
}

void goForwardFast(Robot& robot, EndablePID&& pidDistance, SimplePID&& pidHeading, double fastDistance, double slowdownDistance, double targetHeading) {
    robot.drive->resetDistance();
    robot.drive->setEffort(1,1);
    while (robot.drive->getDistance() < fastDistance) pros::delay(10);
    
    double targetDistance = slowdownDistance + (fastDistance - robot.drive->getDistance());
    goForwardU(robot, std::move(pidDistance), std::move(pidHeading), targetDistance, targetHeading);
}

// Go forwards some distance while maintaining heading
// Return error
double goForwardU(Robot& robot, EndablePID&& pidDistance, SimplePID&& pidHeading, double distance, double targetHeading) {
    
    setHeading(robot, targetHeading);

    robot.drive->resetDistance();

    constexpr int32_t MAX_TIMEOUT = 3500;
    int32_t startTime = pros::millis();

    // FULL EXAMPLE FUNCTION
    while (!pidDistance.isCompleted()/*  && pros::millis() - startTime < MAX_TIMEOUT*/) {

        double baseVelocity = pidDistance.tick(distance - robot.drive->getDistance());
        double headingError = deltaInHeading(targetHeading, robot.localizer->getHeading());
        //pros::lcd::print(0, "Heading error: %f", headingError);
        //pros::lcd::print(1, "Target heading: %f", targetHeading);
        //pros::lcd::print(2, "Current heading: %f", robot.localizer->getHeading());
        double deltaVelocity = pidHeading.tick(headingError);

        double left = baseVelocity - deltaVelocity;
        double right = baseVelocity + deltaVelocity;

        printf("%.2f %.2f\n", left, right);

        robot.drive->setEffort(left, right);

        pros::delay(10);
    }
    if (pidDistance.stopMotors) robot.drive->stop();
    return distance - robot.drive->getDistance();
}

// Turn to some given heading: left is positive
void goTurnU(Robot& robot, EndablePID&& pidHeading, double absoluteHeading) {
    while(!pidHeading.isCompleted()) {
        double headingError = deltaInHeading(absoluteHeading, robot.localizer->getHeading());
        double turnVelocity = pidHeading.tick(headingError);

        double left = -turnVelocity;
        double right = turnVelocity;
        robot.drive->setEffort(left, right);
        

        pros::delay(10);
    }
    
    robot.drive->stop();
}


// Have the robot move in a curve starting from startTheta to endTheta given the radius of curvature about a point that the robot's center would travel around
// A negative radius reverse
void goCurveU(Robot& robot, EndablePID&& pidDistance, SimplePID&& pidCurve, double startTheta, double endTheta, double radius) {
    
    bool reverse = radius < 0;
    radius = fabs(radius);

    double deltaTheta = deltaInHeading(endTheta, startTheta);
    
    double totalDistance = fabs(deltaTheta) * radius;
    double HTW = robot.drive->TRACK_WIDTH / 2.0;
    double slowerWheelRatio = (radius - HTW) / (radius + HTW);

    double largerDistanceTotal = (radius + HTW) * fabs(deltaTheta);

    robot.drive->resetDistance();

    while (!pidDistance.isCompleted()) {
        double largerDistanceCurrent = (deltaTheta > 0 != reverse) ? robot.drive->getRightDistance() : robot.drive->getLeftDistance();
        largerDistanceCurrent = fabs(largerDistanceCurrent);
        double distanceError = largerDistanceTotal - largerDistanceCurrent;

        double fasterWheelSpeed = pidDistance.tick(distanceError);
        double slowerWheelSpeed = fasterWheelSpeed * slowerWheelRatio;

        double targetTheta = startTheta + deltaTheta * (largerDistanceCurrent / largerDistanceTotal);
        pros::lcd::print(0, "Target degrees %f", getDegrees(targetTheta));
        double headingError = deltaInHeading(targetTheta, robot.localizer->getHeading());
        double headingCorrection = pidCurve.tick(headingError);

        double left, right;
        if (deltaTheta < 0 != reverse) {
            left = fasterWheelSpeed;
            right = slowerWheelSpeed;
        } else {
            left = slowerWheelSpeed;
            right = fasterWheelSpeed;
        }

        if (reverse) {
            left *= -1;
            right *= -1;
        }

        // IMU PID Correction:
        left -= headingCorrection; 
        right += headingCorrection;

        robot.drive->setEffort(left, right);

        pros::delay(10);
    }

    if (pidDistance.stopMotors) robot.drive->stop();
}


// go to (x,y) through concurrently aiming at (x,y) and getting as close to it as possible
void goToPoint(Robot& robot, EndablePID&& pidDistance, SimplePID&& pidHeading, double goalX, double goalY) {

    double startX = robot.localizer->getX();
    double startY = robot.localizer->getY();

    double recalculateHeading = true;
    double targetHeading = headingToPoint(startX, startY, goalX, goalY);

    while(!pidDistance.isCompleted()){

        double x = robot.localizer->getX();
        double y = robot.localizer->getY();
        double h = robot.localizer->getHeading();

        double otherX = x + cos(h);
        double otherY = y + sin(h);

        double currentDistance = -distancePointToLine(goalX, goalY, x, y, otherX, otherY); 
        if (currentDistance < 12) recalculateHeading = false;
        if (recalculateHeading) targetHeading = headingToPoint(x, y, goalX, goalY);

        // pros::lcd::clear();
        // pros::lcd::print(0, "%.2f", currentDistance);      
        // pros::lcd::print(1, "goal %.2f %.2f", goalX, goalY);
        // pros::lcd::print(2, "current %.2f %.2f", x, y);`
        // pros::lcd::print(3, "other %.2f %.2f", otherX, otherY);
        double baseVelocity = pidDistance.tick(currentDistance);

        double headingError = deltaInHeading(targetHeading, robot.localizer->getHeading());
        double deltaVelocity = pidHeading.tick(headingError);

        double left = baseVelocity - deltaVelocity;
        double right = baseVelocity + deltaVelocity;
        robot.drive->setEffort(left, right);

        pros::delay(10);
    }
    
    robot.drive->stop();
}

void turnToPoint(Robot& robot, EndablePID&& pidHeading, double goalX, double goalY) {

    double startX = robot.localizer->getX();
    double startY = robot.localizer->getY();

    double targetHeading = headingToPoint(startX, startY, goalX, goalY);

    goTurnU(robot, std::move(pidHeading), targetHeading);

}