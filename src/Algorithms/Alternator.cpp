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
    currentPeriod = 0;
    endCount = 0;
}

bool Alternator::tick() {


    if (currentPeriod >= N - 1) {
        endCount++;
        return true;
    }

    bool result = isA;

    i += 1;
    if (isA && i == A) {
        i = 0;
        isA = false;
        currentPeriod++;
    } else if (!isA && i == B) {
        i = 0;
        isA = true;
    }

    return result;
}

bool Alternator::isDone() {
    return endCount >= end;
}