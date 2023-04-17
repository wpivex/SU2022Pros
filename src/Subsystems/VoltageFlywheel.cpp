#include "Subsystems/Flywheel/VoltageFlywheel.h"
#include "Algorithms/ConversionData.h"

VoltageFlywheel::VoltageFlywheel(std::initializer_list<int8_t> flywheelMotors, std::vector<DataPoint> voltRpmData, std::vector<DataPoint> rpmDistanceFlapDownData, std::vector<DataPoint> rpmDistanceFlapUpData, double startSpeed, double gainConstant):
    Flywheel(flywheelMotors, voltRpmData, rpmDistanceFlapDownData, rpmDistanceFlapUpData, startSpeed),
    gain(gainConstant)
{}

void VoltageFlywheel::maintainVelocityTask() {

    if (isOn) return;
    isOn = true;
    
    while (true) {

        motors.move_voltage(getTargetVelocity() / 3600.0 * 1000); // millivolts
        pros::delay(10);
    }

}

double VoltageFlywheel::getNextMotorVoltage(double currentRPM) {
    return rpmToVolt(data, currentRPM);
}