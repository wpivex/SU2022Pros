#pragma once

#include "main.h"
#include "pros/misc.h"
#include "config.h"

class ControllerSM {

private:
    const static int NUM_BUTTONS = 12;
    pros::controller_digital_e_t buttons[NUM_BUTTONS] = {
        DIGITAL_A, DIGITAL_B, DIGITAL_X, DIGITAL_Y,
        DIGITAL_UP, DIGITAL_DOWN, DIGITAL_LEFT, DIGITAL_RIGHT,
        DIGITAL_L1, DIGITAL_L2, DIGITAL_R1, DIGITAL_R2};

    bool prevButtonState[NUM_BUTTONS] = {false};

    int get(pros::controller_digital_e_t button);

    pros::Controller controller;
    

public:

    ControllerSM():
        controller(pros::E_CONTROLLER_MASTER)
    {}

    void updateButtonState();

    bool pressing(pros::controller_digital_e_t button);
    bool pressed(pros::controller_digital_e_t button);
    bool released(pros::controller_digital_e_t button);

    double getAxis(pros::controller_analog_e_t axis);
};

