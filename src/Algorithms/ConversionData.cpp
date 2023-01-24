#include "Algorithms/ConversionData.h"

float voltToRpm(std::vector<DataPoint>& data, float volt) {
    
    int lowerBound = 0;
    while (lowerBound < data.size() - 2 && data[lowerBound+1].volt < volt) lowerBound++;
    
    float percent = (volt - data[lowerBound].volt) / (data[lowerBound+1].volt - data[lowerBound].volt);
    return data[lowerBound].rpm + (data[lowerBound+1].rpm - data[lowerBound].rpm) * percent;

}

float rpmToVolt(std::vector<DataPoint>& data, float rpm) {

    int lowerBound = 0;
    while (lowerBound < data.size() - 2 && data[lowerBound+1].rpm < rpm) lowerBound++;
    
    float percent = (rpm - data[lowerBound].rpm) / (data[lowerBound+1].rpm - data[lowerBound].rpm);
    return data[lowerBound].volt + (data[lowerBound+1].volt - data[lowerBound].volt) * percent;

}