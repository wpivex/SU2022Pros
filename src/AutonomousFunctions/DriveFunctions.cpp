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

// Turn to some given heading
void goTurnU(Robot& robot, SimplePID&& pidHeading, float absoluteHeading) {
    // TODO
}

// Go to some x position by driving forwards or backwards. Works best when roughly perpendicular to x axis
void goToX(Robot& robot, EndablePID&& pidDistance, float xcoord, float targetHeading) {
    setHeading(robot, targetHeading);
    // TODO
}

// Go to some y position by driving forwards or backwards. Works best when roughly perpendicular to y axis
void goToY(Robot& robot, EndablePID&& pidDistance, float ycoord, float targetHeading) {
    setHeading(robot, targetHeading);
    // TODO
}

// Go as close to some line (defined by two points) as possible
// Essentially goForwardU but with ending control from odom
// Line defined by (x1, y1) to (x2, y2) in inches
void goForwardToLineU(Robot& robot, EndablePID&& pidDistance, SimplePID&& pidHeading,
        float x1, float y1, float x2, float y2, float targetHeading) {
    
    setHeading(robot, targetHeading);
    // TODO
}


// go to (x,y) through concurrently aiming at (x,y) and getting as close to it as possible
void goToPoint(Robot& robot, EndablePID&& pidDistance, SimplePID&& pidHeading, float xcoord, float ycoord) {
    // TODO
}