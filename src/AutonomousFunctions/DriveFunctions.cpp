

#include "AutonomousFunctions/DriveFunctions.h"
#include "misc/MathUtility.h"
#include "pros/rtos.hpp"

// Check if targetHeading was set to default, in which case maintain the current heading and update targetHeading value
inline void setHeading(Robot& robot, float& targetHeading) {
    if (targetHeading == MAINTAIN_CURRENT_HEADING) targetHeading = robot.localizer->getHeading();
}

// Go forwards for some time while maintaining heading
void goForwardTimedU(Robot& robot, SimplePID&& pidHeading, float timeSeconds, float targetEffort, float targetHeading) {
    
    setHeading(robot, targetHeading);

    uint32_t endTime = pros::millis() + timeSeconds * 1000;

    while (pros::millis() < endTime) {

        float headingError = deltaInHeading(targetHeading, robot.localizer->getHeading());
        float deltaVelocity = pidHeading.tick(headingError);
        
        float left = targetEffort - deltaVelocity;
        float right = targetEffort + deltaVelocity;
        robot.drive->setEffort(left, right);

        pros::delay(10);
    }

    robot.drive->stop();
}

// Go forwards some distance while maintaining heading
void goForwardU(Robot& robot, EndablePID&& pidDistance, SimplePID&& pidHeading, float distance, float targetHeading) {
    
    setHeading(robot, targetHeading);

    robot.drive->resetDistance();

    constexpr int32_t MAX_TIMEOUT = 3500;
    int32_t startTime = pros::millis();

    // FULL EXAMPLE FUNCTION
    while (!pidDistance.isCompleted()/*  && pros::millis() - startTime < MAX_TIMEOUT*/) {

        float baseVelocity = pidDistance.tick(distance - robot.drive->getDistance());
        float headingError = deltaInHeading(targetHeading, robot.localizer->getHeading());
        pros::lcd::print(0, "%f", headingError);
        float deltaVelocity = pidHeading.tick(headingError);

        float left = baseVelocity - deltaVelocity;
        float right = baseVelocity + deltaVelocity;
        robot.drive->setEffort(left, right);

        pros::delay(10);
    }
    if (pidDistance.stopMotors) robot.drive->stop();
}

// Turn to some given heading: left is positive
void goTurnU(Robot& robot, EndablePID&& pidHeading, float absoluteHeading) {
    while(!pidHeading.isCompleted()) {
        float headingError = deltaInHeading(absoluteHeading, robot.localizer->getHeading());
        float turnVelocity = pidHeading.tick(headingError);

        float left = -turnVelocity;
        float right = turnVelocity;
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
void goToPoint(Robot& robot, EndablePID&& pidDistance, SimplePID&& pidHeading, float goalX, float goalY) {

    double startX = robot.localizer->getX();
    double startY = robot.localizer->getY();

    double targetHeading = headingToPoint(startX, startY, goalX, goalY);
    double targetDistance = getDistance(startX, startY, goalX, goalY);

    while(!pidDistance.isCompleted()){

        double x = robot.localizer->getX();
        double y = robot.localizer->getY();

        float currentDistance = distanceToPointProjection(x, y, startX, startY, goalX, goalY);        
        float baseVelocity = pidDistance.tick(targetDistance - currentDistance);

        float headingError = deltaInHeading(targetHeading, robot.localizer->getHeading());
        float deltaVelocity = pidHeading.tick(headingError);

        float left = baseVelocity + deltaVelocity;
        float right = baseVelocity - deltaVelocity;
        robot.drive->setEffort(left, right);

        pros::delay(10);
    }
    
    robot.drive->stop();
}

void turnToPoint(Robot& robot, EndablePID&& pidHeading, float goalX, float goalY) {

    double startX = robot.localizer->getX();
    double startY = robot.localizer->getY();

    double targetHeading = headingToPoint(startX, startY, goalX, goalY);

    goTurnU(robot, std::move(pidHeading), targetHeading);

}