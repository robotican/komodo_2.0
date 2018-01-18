//
// Created by Eli Eli on 18/11/2017.
//

#ifndef RIC_INTERFACE_TIMER_H
#define RIC_INTERFACE_TIMER_H

#include <chrono>

class Timer
{
private:
    std::chrono::steady_clock::time_point begin_;
    std::chrono::steady_clock::time_point end_;
    int micro_secs_ = 0;
    bool started_ = false;

public:
    void reset();
    void startMeasure();
    void endMeasure();
    void startTimer(int micro_secs);
    bool isFinished();
    long long int elaspedTimeSec();
    long long int elapsedTimeMilliSec();
    long long int elapsedTimeNanoSec();
    long long int elapsedTimeMicroSec();
};

#endif //RIC_INTERFACE_TIMER_H
