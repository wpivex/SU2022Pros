#pragma once
#include <vector>

typedef struct DataPoint {
    float rpm, volt;
} DataPoint;

float voltToRpm(std::vector<DataPoint>& data, float volt);
float rpmToVolt(std::vector<DataPoint>& data, float rpm);