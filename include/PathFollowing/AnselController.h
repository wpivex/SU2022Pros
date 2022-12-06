#pragma once
#include "Controller.h"

class AnselController : public Controller {

public:
    void runSegment(std::vector<const Waypoint>& path) override;

private:
    Waypoint getTargetWaypoint();


};