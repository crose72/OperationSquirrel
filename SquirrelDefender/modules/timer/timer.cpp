/********************************************************************************
 * @file    timer.cpp
 * @author  Cameron Rose
 * @date    6/7/2023
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
Timer::Timer(void) {}

/********************************************************************************
 * Function: ~Timer
 * Description: Class destructor
 ********************************************************************************/
Timer::~Timer(void) {}

/********************************************************************************
 * Function: calc_loop_start_time
 * Description: Get loop start time.
 ********************************************************************************/
void Timer::calc_loop_start_time(void)
{
    loop_start_time = std::chrono::steady_clock::now();
}

/********************************************************************************
 * Function: calc_loop_end_time
 * Description: Get loop end time.
 ********************************************************************************/
void Timer::calc_loop_end_time(void)
{
    loop_end_time = std::chrono::steady_clock::now();
}

/********************************************************************************
 * Function: loop_rate_controller
 * Description: If loop finished early wait until the desired frequency is
 *              achieved before executing the next loop.
 ********************************************************************************/
void Timer::loop_rate_controller(void)
{
    calc_loop_end_time();

    std::chrono::milliseconds loop_duration = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end_time - loop_start_time);
    while (loop_duration < std::chrono::milliseconds(25))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // wait in small increments
        loop_end_time = std::chrono::steady_clock::now();
        loop_duration = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end_time - loop_start_time);
    }
}
