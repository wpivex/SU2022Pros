#include "PathFollowing/PathFollower.h"

// Blocking function that runs a segment of the path given the index and the controller
void PathFollower::runSegment(int index) {
    controller->runSegment(path[index]);
}