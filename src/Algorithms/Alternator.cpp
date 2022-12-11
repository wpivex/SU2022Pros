#include "Algorithms/Alternator.h"

Alternator::Alternator(int numberOfTimes, int onPeriod, int offPeriod, int endPeriod):
    N(numberOfTimes),
    A(onPeriod),
    B(offPeriod),
    end(endPeriod)
{
    reset();
}

void Alternator::reset() {
    i = 0;
    isA = true;
    currentPeriod = 1;
    endCount = 0;
}

bool Alternator::tick() {


    if (currentPeriod >= N) {
        endCount++;
        return true;
    }

    bool result = isA;

    i += 1;
    if (isA && i == A) {
        i = 0;
        isA = false;
    } else if (!isA && i == B) {
        i = 0;
        isA = true;
        currentPeriod++;
    }

    return result;
}

bool Alternator::isDone() {
    return endCount >= end;
}