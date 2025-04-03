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
Timer::Timer(std::chrono::milliseconds loop_rate_desired) : loop_rate_desired(loop_rate_desired) {}

/********************************************************************************
 * Function: ~Timer
 * Description: Class destructor
 ********************************************************************************/
Timer::~Timer(void) {}

/********************************************************************************
 * Function: start_time
 * Description: Get loop start time.
 ********************************************************************************/
void Timer::start_time(void)
{
    loop_start_time = std::chrono::steady_clock::now();
}

/********************************************************************************
 * Function: end_time
 * Description: Get loop end time.
 ********************************************************************************/
void Timer::end_time(void)
{
    loop_end_time = std::chrono::steady_clock::now();
}

/********************************************************************************
 * Function: wait
 * Description: If loop finished early wait until the desired frequency is
 *              achieved before executing the next loop.
 ********************************************************************************/
void Timer::wait(void)
{
    end_time();

    std::chrono::milliseconds loop_duration = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end_time - loop_start_time);
    while (loop_duration < loop_rate_desired)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // wait in small increments
        loop_end_time = std::chrono::steady_clock::now();
        loop_duration = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end_time - loop_start_time);
    }
}
