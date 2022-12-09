#pragma once
#include "Controller.h"

class AnselController : public Controller {

public:
    void runSegment(std::vector<Waypoint>& path) override;

private:
    Waypoint getTargetWaypoint(std::vector<Waypoint>& path, int& closestIndex, Waypoint& currentPosition);


};