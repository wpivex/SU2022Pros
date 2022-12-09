#include "PathFollowing/AnselController.h"
#include "AutonomousFunctions/DriveFunctions.h"
#include "Algorithms/SimplePID.h"
#include "Algorithms/SingleBoundedPID.h"

#define LOOKAHEAD_DELTA 10 // in terms of waypoint indices
#define LOOKAHEAD_SEARCH_RANGE 10 // the range to look for the closest/target waypoint
#define BASE_EFFORT 0.2 // base speed (0-1)
#define HEADING_KP 7 // how much to compensate for heading error to target

Waypoint AnselController::getTargetWaypoint(std::vector<Waypoint>& path, int& closestIndex, Waypoint& currentPosition) {

    double closestDistance = distance(currentPosition, path[closestIndex]);
    int maxIndex = closestIndex + LOOKAHEAD_SEARCH_RANGE;
    for (int i = closestIndex + 1; i < path.size() && i < maxIndex; i++) {
        double dist = distance(currentPosition, path[i]);
        if (dist < closestDistance) {
            closestIndex = i;
            closestDistance = dist;
        }
    }
    int targetIndex = fmin(closestIndex + LOOKAHEAD_DELTA, path.size() - 1);
    return path[targetIndex];

}


// Blocking method to run pure pursuit on the given path
// When last lookahead point is reached, call goToPoint() instead
void AnselController::runSegment(std::vector<Waypoint>& path) {

    int closestIndex = 0;
    Waypoint targetPosition;
    while (true) {

        Waypoint currentPosition = {robot->localizer->getX(), robot->localizer->getY()};
        double currentHeading = robot->localizer->getHeading();

        targetPosition = getTargetWaypoint(path, closestIndex, currentPosition);

        if (closestIndex == path.size() - 1) break;

        double headingToTarget = thetaBetweenWaypoints(currentPosition, targetPosition);
        double headingError = deltaInHeading(headingToTarget, currentHeading);

        double leftEffort = BASE_EFFORT + HEADING_KP * headingError;
        double rightEffort = BASE_EFFORT - HEADING_KP * headingError;

        robot->drive->setEffort(leftEffort, rightEffort);

        pros::delay(10);
    }
    
    // go directly to last point once lookahead target becomes the last waypoint
    goToPoint(*robot, SingleBoundedPID({1,0,0}), SimplePID({1,0,0}), targetPosition.x, targetPosition.y);
}