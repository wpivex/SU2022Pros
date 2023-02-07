#pragma once

#include "Algorithms/SimplePID.h"
#include "Algorithms/EndablePID.h"
#include "Subsystems/Robot.h"

#define MAINTAIN_CURRENT_HEADING 12345 // by default, target heading is simply the current heading the robot is at

// Go forwards for some time while maintaining heading
void goForwardTimedU(Robot& robot, SimplePID&& pidHeading, float timeSeconds, float targetEffort, float targetHeading = MAINTAIN_CURRENT_HEADING);

// Go forwards some distance while maintaining heading
void goForwardU(Robot& robot, EndablePID&& pidDistance, SimplePID&& pidHeading, float distance, float targetHeading = MAINTAIN_CURRENT_HEADING);

// Turn to some given heading
void goTurnU(Robot& robot, EndablePID&& pidHeading, float absoluteHeading);

void goTurnEncoder(Robot& robot, EndablePID&& pidDistance, float theta);

// Have the robot move in a curve starting from startTheta to endTheta given the radius of curvature about a point that the robot's center would travel around
// A negative radius means moving backwards
void goCurveU(Robot& robot, EndablePID&& pidDistance, SimplePID&& pidCurve, double startTheta, double endTheta, double radius);

// Go to some x position by driving forwards or backwards. Works best when roughly perpendicular to x axis
void goToX(Robot& robot, EndablePID&& pidDistance, float xcoord, float targetHeading = MAINTAIN_CURRENT_HEADING);

// Go to some y position by driving forwards or backwards. Works best when roughly perpendicular to y axis
void goToY(Robot& robot, EndablePID&& pidDistance, float ycoord, float targetHeading = MAINTAIN_CURRENT_HEADING);

// Go as close to some line (defined by two points) as possible
// Essentially goForwardU but with ending control from odom
// Line defined by (x1, y1) to (x2, y2) in inches
void goForwardToLineU(Robot& robot, EndablePID&& pidDistance, SimplePID&& pidHeading,
    float x1, float y1, float x2, float y2, float targetHeading = MAINTAIN_CURRENT_HEADING);


// go to (x,y) through concurrently aiming at (x,y) and getting as close to it as possible
void goToPoint(Robot& robot, EndablePID&& pidDistance, SimplePID&& pidHeading, float xcoord, float ycoord);
void turnToPoint(Robot& robot, EndablePID&& pidHeading, float xcoord, float ycoord);