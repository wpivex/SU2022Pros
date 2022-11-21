#include "misc/ButtonSM.h"

void ButtonSM::updateButtonState() {
    for (int i = 0; i < NUM_BUTTONS; i++) {
        prevButtonState[i] = pressing(buttons[i]); 
    }
}

bool ButtonSM::pressing(pros::controller_digital_e_t button) {
    return controller.get_digital(button);
}

bool ButtonSM::pressed(pros::controller_digital_e_t button) {
    return pressing(button) && !prevButtonState[get(button)];
}

bool ButtonSM::released(pros::controller_digital_e_t button) {
    return !pressing(button) && prevButtonState[get(button)];
}

int ButtonSM::get(pros::controller_digital_e_t button) {
    switch (button) {
        case DIGITAL_A:
            return 0;
        case DIGITAL_B:
            return 1;
        case DIGITAL_X:
            return 2;
        case DIGITAL_Y:
            return 3;
        case DIGITAL_UP:
            return 4;
        case DIGITAL_DOWN:
            return 5;
        case DIGITAL_LEFT:
            return 6;
        case DIGITAL_RIGHT:
            return 7;
        case DIGITAL_L1:
            return 8;
        case DIGITAL_L2:
            return 9;
        case DIGITAL_R1:
            return 10;
        case DIGITAL_R2:
            return 11;
        default:
            return -1;
    }
}