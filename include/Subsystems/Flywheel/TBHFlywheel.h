#include "Subsystems/Flywheel/Flywheel.h"

class TBHFlywheel : public Flywheel {

private:

    double gain;
    double output = 0;
    double prevError = 0;
    double tbh;


public:

    TBHFlywheel(std::initializer_list<int8_t> flywheelMotors, std::vector<DataPoint> voltRpmData, std::vector<DataPoint> rpmDistanceFlapDownData, std::vector<DataPoint> rpmDistanceFlapUpData, double startSpeed, double gainConstant);
 
    double getNextMotorVoltage(double currentRPM) override;
};