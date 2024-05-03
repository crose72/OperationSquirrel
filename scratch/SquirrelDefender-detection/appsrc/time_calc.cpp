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
 * Function: calcStartTimeMS
 * Description: Calculate program start time in milliseconds.
 ********************************************************************************/
void calcStartTimeMS(void)
{
    // Get the start time
    startTime = std::chrono::high_resolution_clock::now();
}

/********************************************************************************
 * Function: calcElapsedTime
 * Description: Calculate program elapsed time in milliseconds.
 ********************************************************************************/
void calcElapsedTime(void)
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