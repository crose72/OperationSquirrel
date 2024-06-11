/********************************************************************************
 * @file    time_calc.cpp
 * @author  Cameron Rose
 * @date    6/7/2023
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
float app_elapsed_time = 0.0f;
const float time_step = 0.025f;
std::chrono::time_point<std::chrono::steady_clock> start_time = std::chrono::steady_clock::now();
std::chrono::time_point<std::chrono::steady_clock> current_time = std::chrono::steady_clock::now();
std::chrono::duration<float, std::milli> elapsed_time((float)0.0);

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
TimeCalc::TimeCalc(void) {}

/********************************************************************************
 * Function: ~TimeCalc
 * Description: Class destructor
 ********************************************************************************/
TimeCalc::~TimeCalc(void) {}

/********************************************************************************
 * Function: calc_app_start_time
 * Description: Get program start time
 ********************************************************************************/
void TimeCalc::calc_app_start_time(void)
{
    start_time = std::chrono::steady_clock::now();
}

/********************************************************************************
 * Function: calc_app_end_time
 * Description: Get program end time
 ********************************************************************************/
void TimeCalc::calc_app_end_time(void)
{
    std::chrono::time_point<std::chrono::steady_clock> app_end_time = std::chrono::steady_clock::now();
}

/********************************************************************************
 * Function: calc_loop_start_time
 * Description: Get loop start time.
 ********************************************************************************/
void TimeCalc::calc_loop_start_time(void)
{
    loop_start_time = std::chrono::steady_clock::now();
}

/********************************************************************************
 * Function: calc_loop_end_time
 * Description: Get loop end time.
 ********************************************************************************/
void TimeCalc::calc_loop_end_time(void)
{
    loop_end_time = std::chrono::steady_clock::now();
}

/********************************************************************************
 * Function: loop_rate_controller
 * Description: If loop finished early wait until the desired frequency is
 *              achieved before executing the next loop.
 ********************************************************************************/
void TimeCalc::loop_rate_controller(void)
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

/********************************************************************************
 * Function: calc_elapsed_time
 * Description: Calculate program elapsed time in milliseconds.
 ********************************************************************************/
void TimeCalc::calc_elapsed_time(void)
{
    if (first_loop_after_start == true)
    {
        app_elapsed_time = 0.0f;
    }
    else
    {
        // Get the current timestamp
        current_time = std::chrono::steady_clock::now();

        // Calculate the elapsed time since the start of the program
        elapsed_time = current_time - start_time;

        // Convert elapsed time to seconds with millisecond precision
        float app_elapsed_time_tmp = elapsed_time.count() / 1000.0f;

        // Truncate the number to three decimal places
        app_elapsed_time = (std::floor(app_elapsed_time_tmp * 1000.0f) / 1000.0f - time_step);

        // Print program run time
        // PrintPass::c_printf("Elapsed Time: %0.3f\n", app_elapsed_time);
    }
}
