/********************************************************************************
 * @file    timer.h
 * @date    1/22/2025
 ********************************************************************************/
#ifndef TIMER_H
#define TIMER_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include <chrono>
#include <thread>
#include <cmath>

/********************************************************************************
 * Typedefs / Enums / Structs
 ********************************************************************************/

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
class Timer
{
public:
    Timer(std::chrono::milliseconds mDesiredRate);
    Timer();
    ~Timer();

    std::chrono::milliseconds getLoopTime();
    std::chrono::milliseconds getDur(void);
    void start(void);
    void stop(void);
    void wait(void);

private:
    std::chrono::time_point<std::chrono::steady_clock> mStartTime;
    std::chrono::time_point<std::chrono::steady_clock> mEndTime;
    std::chrono::milliseconds mDesiredRate;
    std::chrono::milliseconds mLoopTime;
};

#endif // TIMER_H
