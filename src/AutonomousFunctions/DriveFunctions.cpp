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
        
        float left = targetEffort + deltaVelocity;
        float right = targetEffort - deltaVelocity;
        robot.drive->setEffort(left, right);

        pros::delay(10);
    }

    robot.drive->stop();
}

// Go forwards some distance while maintaining heading
void goForwardU(Robot& robot, EndablePID&& pidDistance, SimplePID&& pidHeading, float distance, float targetHeading) {
    
    setHeading(robot, targetHeading);

    robot.drive->resetDistance();

    // FULL EXAMPLE FUNCTION
    while (!pidDistance.isCompleted()) {

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
        //written out in variables for ease of debugging. can shorten later
        // also I'm pretty sure the signs should be flipped but I'm keeping it consistent for now.

        pros::delay(10);
    }
    
    robot.drive->stop();
}

void goTurnEncoder(Robot& robot, EndablePID&& pidDistance, float theta) {

    robot.drive->resetDistance();

    float totalDistance = theta * robot.drive->TRACK_WIDTH / 2;

    while (!pidDistance.isCompleted()) {

        float currentDistance = (robot.drive->getRightDistance() - robot.drive->getLeftDistance()) / 2;
        float speed = pidDistance.tick(totalDistance - currentDistance);
        robot.drive->setEffort(-speed, speed);
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


        left -= headingCorrection;
        right += headingCorrection;

        robot.drive->setEffort(left, right);

        pros::delay(10);
    }

    if (pidDistance.stopMotors) robot.drive->stop();
}

// Go to some x position by driving forwards or backwards. Works best when roughly perpendicular to x axis
void goToX(Robot& robot, EndablePID&& pidDistance, SimplePID&& pidHeading, float xcoord) {
    float targetHeading = robot.localizer->getHeading();

    robot.drive->resetDistance();

    while (!pidDistance.isCompleted()) {
        float distance = (xcoord-robot.localizer->getY())*cosf((targetHeading+(robot.localizer->getHeading()))/2);
        // something is off but my brain stopped going brr when my stomach decided i was hungry. shall return

        float baseVelocity = pidDistance.tick(distance - robot.drive->getDistance());
        float headingError = deltaInHeading(targetHeading, robot.localizer->getHeading());
        float deltaVelocity = pidHeading.tick(headingError);

        float left = baseVelocity + deltaVelocity;
        float right = baseVelocity - deltaVelocity;
        robot.drive->setEffort(left, right);

        pros::delay(10);
    }
    
    robot.drive->stop();
}

// Go to some y position by driving forwards or backwards. Works best when roughly perpendicular to y axis
void goToY(Robot& robot, EndablePID&& pidDistance, SimplePID&& pidHeading, float ycoord, float targetHeading) {
    robot.drive->resetDistance();

    while (!pidDistance.isCompleted()) {
        float distance = (ycoord-robot.localizer->getY())*cosf((targetHeading+(robot.localizer->getHeading()))/2);
        
        float baseVelocity = pidDistance.tick(distance - robot.drive->getDistance());
        float headingError = deltaInHeading(targetHeading, robot.localizer->getHeading());
        float deltaVelocity = pidHeading.tick(headingError);

        float left = baseVelocity + deltaVelocity;
        float right = baseVelocity - deltaVelocity;
        robot.drive->setEffort(left, right);

        pros::delay(10);
    }
    
    robot.drive->stop();
}

// Go as close to some line (defined by two points) as possible
// Essentially goForwardU but with ending control from odom
// Line defined by (x1, y1) to (x2, y2) in inches

/*
LEAVING THIS HERE UNTIL FUNCTION IS BUG FREE

find equation for line
find equation for robot movement
find where they intersect
calculate distance to point
drive to point while controlling heading

slope = (y2-y1)/(x2-x1)

y - y1 = slope(x - x1)
roboSlope= tan(robot.localizer->getHeading());
y - robot.localizer->getY() = roboSlope(x - robot.localizer->getX())

slope(x - x1) + y1 = roboSlope(x - robot.localizer->getX()) + robot.localizer->getY()
slope*x - slope*x1 + y1 = roboSlope*x - roboSlope*robot.localizer->getX()) + robot.localizer->getY()
slope*x -  roboSlope*x  = - roboSlope*robot.localizer->getX()) + robot.localizer->getY() + slope*x1 - y1
x  = (-roboSlope*robot.localizer->getX()) + robot.localizer->getY() + slope*x1 - y1)/ (slope - roboSlope)
*/
void goForwardToLineU(Robot& robot, EndablePID&& pidDistance, SimplePID&& pidHeading,
        float x1, float y1, float x2, float y2, float targetHeading) {
    
    setHeading(robot, targetHeading);

    while(!pidDistance.isCompleted()){
        float lineSlope = (y2-y1)/(x2-x1);
        float roboSlope= tan(robot.localizer->getHeading());

        float desX = ((-roboSlope*robot.localizer->getX()) + robot.localizer->getY() + (lineSlope*x1) - y1)/ (lineSlope - roboSlope);
        float desY = (lineSlope*(desX - x1)) + y1;

        float distance = sqrtf(pow((desX-robot.localizer->getX()),2)+pow((desY-robot.localizer->getY()),2));

        float baseVelocity = pidDistance.tick(distance - robot.drive->getDistance());
        float headingError = deltaInHeading(targetHeading, robot.localizer->getHeading());
        float deltaVelocity = pidHeading.tick(headingError);

        float left = baseVelocity + deltaVelocity;
        float right = baseVelocity - deltaVelocity;
        robot.drive->setEffort(left, right);
    }
    
    robot.drive->stop();
}

// go to (x,y) through concurrently aiming at (x,y) and getting as close to it as possible
void goToPoint(Robot& robot, EndablePID&& pidDistance, SimplePID&& pidHeading, float xcoord, float ycoord) {
    robot.drive->resetDistance();

    while(!pidDistance.isCompleted()){
        float targetHeading = (atan2f((ycoord-robot.localizer->getY()), (xcoord-robot.localizer->getX())))*180/M_PI;

        float distance = sqrtf(pow((xcoord-robot.localizer->getX()),2)+pow((ycoord-robot.localizer->getY()),2));
        
        float baseVelocity = pidDistance.tick(distance - robot.drive->getDistance());
        float headingError = deltaInHeading(targetHeading, robot.localizer->getHeading());
        float deltaVelocity = pidHeading.tick(headingError);

        float left = baseVelocity + deltaVelocity;
        float right = baseVelocity - deltaVelocity;
        robot.drive->setEffort(left, right);

        pros::delay(10);
    }
    
    robot.drive->stop();
}