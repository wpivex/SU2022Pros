#include "Algorithms/NoPID.h"

// Doesn't matter what the error is. Always run at max speed, stored in KP
double NoPID::tick(double error) {

    handleEndCondition(error);
    
    return goingUp ? (-K.P) : K.P;
}