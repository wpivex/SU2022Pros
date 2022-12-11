#pragma once

/*
State machine that alternates to an active state for N amount of times with A "on" periods and B "off" periods
*/
class Alternator {

private:
    int N, A, B;
    int currentPeriod;
    int i;
    bool isA;

    int end;
    int endCount = 0;

public:

    Alternator(int numberOfTimes, int onPeriod, int offPeriod, int endPeriod = -1);
    void reset();
    bool tick();
    bool isDone();
};