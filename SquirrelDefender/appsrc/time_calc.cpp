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
float app_elapsed_time = (float)0.0;
const float time_step = (float)0.025;
std::chrono::time_point<std::chrono::high_resolution_clock> start_time = std::chrono::high_resolution_clock::now();
std::chrono::time_point<std::chrono::high_resolution_clock> current_time = std::chrono::high_resolution_clock::now();
std::chrono::duration<float, std::milli> elapsed_time(0.0);

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
 * Function: calc_app_start_time
 * Description: Get program start time
 ********************************************************************************/
void TimeCalc::calc_app_start_time(void)
{
    std::chrono::_V2::system_clock::time_point app_start_time = std::chrono::high_resolution_clock::now();
}

/********************************************************************************
 * Function: calc_app_end_time
 * Description: Get program start time
 ********************************************************************************/
void TimeCalc::calc_app_end_time(void)
{
    std::chrono::_V2::system_clock::time_point app_end_time = std::chrono::high_resolution_clock::now();
}

/********************************************************************************
 * Function: calc_loop_start_time
 * Description: Get loop start time.
 ********************************************************************************/
void TimeCalc::calc_loop_start_time(void)
{
    std::chrono::_V2::system_clock::time_point loop_start_time = std::chrono::high_resolution_clock::now();
}

/********************************************************************************
 * Function: calc_loop_end_time
 * Description: Get loop end time.
 ********************************************************************************/
void TimeCalc::calc_loop_end_time(void)
{
    std::chrono::_V2::system_clock::time_point loop_end_time = std::chrono::high_resolution_clock::now();
}

/********************************************************************************
 * Function: loop_rate_controller
 * Description: If loop finished early wait until the desired frequency is
 *              achieved before executing the next loop.
 ********************************************************************************/
void TimeCalc::loop_rate_controller(void)
{
    std::chrono::milliseconds loop_duration = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end_time - loop_start_time);
    while (loop_duration < std::chrono::milliseconds(25))
    {
        /* wait */
    }
}

/********************************************************************************
 * Function: calc_elapsed_time
 * Description: Calculate program elapsed time in milliseconds.
 ********************************************************************************/
void TimeCalc::calc_elapsed_time(void)
{
    if (firstLoopAfterStartup == true)
    {
        app_elapsed_time = (float)0.0;
    }
    else
    {
        // Get the current timestamp
        current_time = std::chrono::high_resolution_clock::now();

        // Calculate the elapsed time since the start of the program
        elapsed_time = current_time - start_time;

        // Convert elapsed time to seconds with millisecond precision
        float app_elapsed_time_tmp = elapsed_time.count() / 1000.0;

        // Truncate the number to three decimal places
        app_elapsed_time = (floor(app_elapsed_time_tmp * 1000.0) / 1000.0 - time_step);

        // Print program run time
        // printf("Elapsed Time: %0.3f\n", app_elapsed_time);
    }
}