//
// Created by Eli Eli on 18/11/2017.
//

#include <ric_interface/timer.h>

void Timer::reset()
{
    std::chrono::steady_clock::time_point zero;
    begin_ = zero;
    end_ = zero;
    micro_secs_ = 0;
    started_ = false;
}


void Timer::startMeasure()
{
    begin_ = std::chrono::steady_clock::now();
}
void Timer::endMeasure()
{
    end_ = std::chrono::steady_clock::now();
}

void Timer::startTimer(int micro_secs)
{
    if (!started_)
    {
        micro_secs_ = micro_secs;
        startMeasure();
        started_ = true;
    }
}

bool Timer::isFinished()
{
    endMeasure();
    if (elapsedTimeMilliSec() >= micro_secs_)
        return true;
    return false;
}

long long int Timer::elaspedTimeSec()
{
    return std::chrono::duration_cast<std::chrono::seconds>(end_ - begin_).count();
}
long long int Timer::elapsedTimeMilliSec()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(end_ - begin_).count();
}
long long int Timer::elapsedTimeNanoSec()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(end_ - begin_).count();
}

long long int Timer::elapsedTimeMicroSec()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(end_ - begin_).count();

}
