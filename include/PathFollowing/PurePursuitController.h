#pragma once
#include "Controller.h"

class PurePursuitController : public Controller {

public:
    void runSegment(std::vector<Waypoint>& path) override;
};