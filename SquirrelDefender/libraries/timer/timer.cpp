/********************************************************************************
 * @file    timer.cpp
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Time calculations.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "timer.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: Timer
 * Description: Class constructor
 ********************************************************************************/
Timer::Timer() {}

/********************************************************************************
 * Function: Timer
 * Description: Class constructor
 ********************************************************************************/
Timer::Timer(std::chrono::milliseconds mDesiredRate) : mDesiredRate(mDesiredRate) {}

/********************************************************************************
 * Function: ~Timer
 * Description: Class destructor
 ********************************************************************************/
Timer::~Timer(void) {}

/********************************************************************************
 * Function: start_time
 * Description: Get loop start time.
 ********************************************************************************/
void Timer::start(void)
{
    mStartTime = std::chrono::steady_clock::now();
}

/********************************************************************************
 * Function: end_time
 * Description: Get loop end time.
 ********************************************************************************/
void Timer::stop(void)
{
    mEndTime = std::chrono::steady_clock::now();
}

/********************************************************************************
 * Function: end_time
 * Description: Get loop end time.
 ********************************************************************************/
std::chrono::milliseconds Timer::getLoopTime(void)
{
    std::chrono::milliseconds mLoopTime = std::chrono::duration_cast<std::chrono::milliseconds>(mEndTime - mStartTime);
    return mLoopTime;
}

/********************************************************************************
 * Function: start_time
 * Description: Get loop start time.
 ********************************************************************************/
std::chrono::milliseconds Timer::getDur(void)
{
    std::chrono::time_point<std::chrono::steady_clock> currTime = std::chrono::steady_clock::now();
    std::chrono::milliseconds timerDur = std::chrono::duration_cast<std::chrono::milliseconds>(currTime - mStartTime);
    return timerDur;
}

/********************************************************************************
 * Function: wait
 * Description: If loop finished early wait until the desired frequency is
 *              achieved before executing the next loop.
 ********************************************************************************/
void Timer::wait(void)
{
    stop();

    std::chrono::milliseconds mLoopTime = std::chrono::duration_cast<std::chrono::milliseconds>(mEndTime - mStartTime);
    while (mLoopTime < mDesiredRate)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // wait in small increments
        mEndTime = std::chrono::steady_clock::now();
        mLoopTime = std::chrono::duration_cast<std::chrono::milliseconds>(mEndTime - mStartTime);
    }
}
