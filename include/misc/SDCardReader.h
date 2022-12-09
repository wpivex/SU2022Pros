#pragma once

#include "main.h"
#include "pros/misc.h"
#include "config.h"

struct Position {
    float x;
    float y;
};

class SDCardReader {
    public:
        std::vector<std::vector<Position*>> readSDCardData(std::string filename);
};