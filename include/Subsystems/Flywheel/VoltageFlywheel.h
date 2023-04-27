#include "Subsystems/Flywheel/Flywheel.h"

class VoltageFlywheel : public Flywheel {
private:
    double gain;

public:

    VoltageFlywheel(std::initializer_list<int8_t> flywheelMotors, std::vector<DataPoint> voltRpmData, std::vector<DataPoint> rpmDistanceFlapDownData, std::vector<DataPoint> rpmDistanceFlapUpData, double startSpeed, double gainConstant);
    
    void maintainVelocityTask() override;
};