#include "PathFollowing/AnselController.h"
#include "AutonomousFunctions/DriveFunctions.h"

#define LOOKAHEAD_DELTA 10 // in terms of waypoint indices
#define LOOKAHEAD_SEARCH_RANGE 10 // the range to look for the closest/target waypoint
#define BASE_SPEED 0.2 // base speed (0-1)
#define HEADING_KP 7 // how much to compensate for heading error to target

void AnselController::getTargetWaypoint(Waypoint& currentPosition) {

    double closestDistance = distance(currentPosition, path[closestIndex]);
    int maxIndex = closestIndex + LOOKAHEAD_SEARCH_RANGE;
    for (int i = closestIndex, i < path.size() && i < maxIndex; i++) {
        
    }

}


// Blocking method to run pure pursuit on the given path
// When last lookahead point is reached, call goToPoint() instead
void AnselController::runSegment(std::vector<Waypoint>& path) {

    Waypoint currentPosition(robot.localizer->getX(), robot.localizer->getY());

}