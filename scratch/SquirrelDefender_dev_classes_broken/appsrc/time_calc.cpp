/********************************************************************************
 * @file    time_calc.cpp
 * @author  Cameron Rose
 * @date    12/27/2023
 * @brief   Time calculations.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "time_calc.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/
float elapsedTimeSeconds = (float)0.0;
const float timeStep = (float)0.025;
std::chrono::time_point<std::chrono::high_resolution_clock> startTime = std::chrono::high_resolution_clock::now();
std::chrono::time_point<std::chrono::high_resolution_clock> currentTime = std::chrono::high_resolution_clock::now();
std::chrono::duration<float, std::milli> elapsedTime(0.0);

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: TimeCalc
 * Description: Class constructor
 ********************************************************************************/
TimeCalc::TimeCalc(void){}

/********************************************************************************
 * Function: ~TimeCalc
 * Description: Class destructor
 ********************************************************************************/
TimeCalc::~TimeCalc(void){}

/********************************************************************************
 * Function: app_start_time
 * Description: Get program start time
 ********************************************************************************/
void TimeCalc::app_start_time(void)
{
    std::chrono::_V2::system_clock::time_point appStartTime = std::chrono::high_resolution_clock::now();
}

/********************************************************************************
 * Function: app_end_time
 * Description: Get program start time
 ********************************************************************************/
void TimeCalc::app_end_time(void)
{
    std::chrono::_V2::system_clock::time_point appEndTime = std::chrono::high_resolution_clock::now();
}

/********************************************************************************
 * Function: loop_start_time
 * Description: Get loop start time.
 ********************************************************************************/
void TimeCalc::loop_start_time(void)
{
    std::chrono::_V2::system_clock::time_point loopStartTime = std::chrono::high_resolution_clock::now();
}

/********************************************************************************
 * Function: loop_end_time
 * Description: Get loop end time.
 ********************************************************************************/
void TimeCalc::loop_end_time(void)
{
    std::chrono::_V2::system_clock::time_point loopEndTime = std::chrono::high_resolution_clock::now();
}

/********************************************************************************
 * Function: loop_rate_controller
 * Description: If loop finished early wait until the desired frequency is
 *              achieved before executing the next loop.
 ********************************************************************************/
void TimeCalc::loop_rate_controller(void)
{
    std::chrono::milliseconds loopDuration = std::chrono::duration_cast<std::chrono::milliseconds>(loopEndTime - loopStartTime);
    while (loopDuration < std::chrono::milliseconds(25))
    {
        /* wait */
    }
}

/********************************************************************************
 * Function: calcElapsedTime
 * Description: Calculate program elapsed time in milliseconds.
 ********************************************************************************/
void TimeCalc::elapsed_time(void)
{
    if (firstLoopAfterStartup == true)
    {
        elapsedTimeSeconds = (float)0.0;
    }
    else
    {
        // Get the current timestamp
        currentTime = std::chrono::high_resolution_clock::now();

        // Calculate the elapsed time since the start of the program
        elapsedTime = currentTime - startTime;

        // Convert elapsed time to seconds with millisecond precision
        float elapsedTimeSecondsTMP = elapsedTime.count() / 1000.0;

        // Truncate the number to three decimal places
        elapsedTimeSeconds = (floor(elapsedTimeSecondsTMP * 1000.0) / 1000.0 - timeStep);

        // Print program run time
        // printf("Elapsed Time: %0.3f\n", elapsedTimeSeconds);
    }
}