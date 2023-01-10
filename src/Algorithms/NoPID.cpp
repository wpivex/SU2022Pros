#include "Algorithms/NoPID.h"

// Doesn't matter what the error is. Always run at max speed, stored in KP
float NoPID::tick(float error) {

    handleEndCondition(error);
    
    return K.P;
}