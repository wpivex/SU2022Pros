#include "Subsystems/Flywheel/Flywheel.h"

class BBFFlywheel : public Flywheel {

private:

    double tolerance;

public:

    BBFFlywheel(std::initializer_list<int8_t> flywheelMotors, std::vector<DataPoint> voltRpmData, double startSpeed, double toleranceRPM);
 
    double getNextMotorVoltage(double currentRPM) override;


};