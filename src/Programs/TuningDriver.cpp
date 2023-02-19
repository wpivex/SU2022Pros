#include "Programs/TuningDriver.h"
#include "misc/ProsUtility.h"
#include "pros/llemu.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"


// note that brain display ranges lines 0-7
void TuningDriver::runDriver() {

    pros::lcd::initialize();

    robot.drive->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

    const int START_LINE = 3;
    int numParams = test->paramValues.size();

    int selectedParam = 0;
    double time = -1;

    while (true) {

        if (controller.pressed(DIGITAL_A)) time = test->run(robot);

        drawAdjustableParameters(START_LINE, numParams, selectedParam, test->paramValues, test->paramNames, time);
        handleControllerInput(numParams, selectedParam, test->paramValues);
        
        
        // Update button state machine for rising and falling edges
        controller.updateButtonState();

        // Enforce minimum polling cycle rate
        pros::delay(10);
    }
}

void TuningDriver::drawAdjustableParameters(int line, int numParams, int selectedParam, std::vector<double>& paramValues, std::vector<std::string>& paramNames, double time) {
    
    // display time for previous run
    pros::lcd::clear_line(0);
    pros::lcd::print(0, "Time: %f", time);

    // display the adjustable parameters
    std::string str;
    for (int i = 0; i < numParams; i++) {
        pros::lcd::clear_line(line);

        if (i == selectedParam) str = "> ";
        else str = "  ";

        str += paramNames[i] + ": " + std::to_string(paramValues[i]);

        pros::lcd::print(line, str.c_str());

        line++;
        }
}

void TuningDriver::handleControllerInput(int numParams, int& selectedParam, std::vector<double>& paramValues) {

    // handle changing which parameter is selected
    if (controller.pressed(DIGITAL_UP) && selectedParam < numParams - 1) {
        selectedParam++;
    }
    else if (controller.pressed(DIGITAL_DOWN) && selectedParam > 0) {
        selectedParam--;
    }

    // handle adjusting the selected parameter
    const double amount = 0.9;
    if (controller.pressed(DIGITAL_LEFT)) {
        paramValues[selectedParam] *= amount;
    }
    else if (controller.pressed(DIGITAL_RIGHT)) {
        paramValues[selectedParam] /= amount;
    }

}