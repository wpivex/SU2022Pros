#include "Subsystems/Flywheel/VoltageFlywheel.h"
#include "Algorithms/ConversionData.h"

VoltageFlywheel::VoltageFlywheel(std::initializer_list<int8_t> flywheelMotors, std::vector<DataPoint> voltRpmData, std::vector<DataPoint> rpmDistanceFlapDownData, std::vector<DataPoint> rpmDistanceFlapUpData, double startSpeed, double gainConstant):
    Flywheel(flywheelMotors, voltRpmData, rpmDistanceFlapDownData, rpmDistanceFlapUpData, startSpeed),
    gain(gainConstant)
{}

void VoltageFlywheel::maintainVelocityTask() {

}