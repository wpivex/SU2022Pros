#include "AutonomousFunctions/DriveFunctions.h"

// Check if targetHeading was set to default, in which case maintain the current heading and update targetHeading value
inline void setHeading(Robot& robot, float& targetHeading) {
    if (targetHeading == MAINTAIN_CURRENT_HEADING) targetHeading = robot.localizer->getHeading();
}


// Go forwards for some time while maintaining heading
void goForwardTimedU(Robot& robot, SimplePID&& pidHeading, float timeSeconds, float targetHeading) {
    setHeading(robot, targetHeading);
    // TODO
}

// Go forwards some distance while maintaining heading
void goForwardU(Robot& robot, EndablePID&& pidDistance, SimplePID&& pidHeading, float distance, float targetHeading) {
    setHeading(robot, targetHeading);

    robot.drive->resetDistance();

    // FULL EXAMPLE FUNCTION
    while (!pidDistance.isCompleted()) {

        float baseVelocity = pidDistance.tick(distance - robot.drive->getDistance());
        float headingError = deltaInHeading(targetHeading, robot.localizer->getHeading());
        float deltaVelocity = pidHeading.tick(headingError);

        float left = baseVelocity + deltaVelocity;
        float right = baseVelocity - deltaVelocity;
        robot.drive->setVelocity(left, right);

        pros::delay(10);
    }
}

// Turn to some given heading: left is positive
void goTurnU(Robot& robot, EndablePID&& pidHeading, float absoluteHeading) {
    while(!pidHeading.isCompleted()){
        float headingError = deltaInHeading(absoluteHeading, robot.localizer->getHeading());
        float turnVelocity = pidHeading.tick(headingError);

        float left = turnVelocity;
        float right = -turnVelocity;
        robot.drive->setVelocity(left, right);
        //written out in variables for ease of debugging. can shorten later
        // also I'm pretty sure the signs should be flipped but I'm keeping it consistent for now.
    }
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
        robot.drive->setVelocity(left, right);

        pros::delay(10);
    }
}

// Go to some y position by driving forwards or backwards. Works best when roughly perpendicular to y axis
void goToY(Robot& robot, EndablePID&& pidDistance, SimplePID&& pidHeading, float ycoord, float targetHeading) {
    setHeading(robot, targetHeading);

    robot.drive->resetDistance();

    while (!pidDistance.isCompleted()) {
        float distance = (ycoord-robot.localizer->getY())*cosf((targetHeading+(robot.localizer->getHeading()))/2);
        
        float baseVelocity = pidDistance.tick(distance - robot.drive->getDistance());
        float headingError = deltaInHeading(targetHeading, robot.localizer->getHeading());
        float deltaVelocity = pidHeading.tick(headingError);

        float left = baseVelocity + deltaVelocity;
        float right = baseVelocity - deltaVelocity;
        robot.drive->setVelocity(left, right);

        pros::delay(10);
    }
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
        robot.drive->setVelocity(left, right);
    }
    // TODO
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
        robot.drive->setVelocity(left, right);

        pros::delay(10);
    }
}