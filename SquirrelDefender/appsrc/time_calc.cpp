/********************************************************************************
 * @file    time_calc.cpp
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Calculate program run time and main loop rate.
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
float g_app_elapsed_time;
float app_elapsed_time_prv;
float g_dt;
bool g_first_loop_after_start;
std::chrono::time_point<std::chrono::steady_clock> start_time;
std::chrono::duration<float, std::milli> elapsed_time((float)0.0);

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: Time
 * Description: Class constructor
 ********************************************************************************/
Time::Time(void) {}

/********************************************************************************
 * Function: ~Time
 * Description: Class destructor
 ********************************************************************************/
Time::~Time(void) {}

/********************************************************************************
 * Function: init
 * Description: Initialize time calculation.
 ********************************************************************************/
bool Time::init(void)
{
    g_app_elapsed_time = (float)0.0;
    g_dt = (float)0.0;
    app_elapsed_time_prv = (float)0.0;
    g_first_loop_after_start = true;
    
    start_time = std::chrono::steady_clock::now();
    return true;
}


/********************************************************************************
 * Function: loop
 * Description: Main time calculation loop.
 ********************************************************************************/
void Time::loop(void)
{
    if (g_first_loop_after_start)
    {
        g_first_loop_after_start = false;
    }
    else if (!g_first_loop_after_start)
    {
        // Get the current timestamp
        std::chrono::time_point<std::chrono::steady_clock> current_time = std::chrono::steady_clock::now();

        // Calculate the elapsed time since the start of the program
        elapsed_time = current_time - start_time;

        // Convert elapsed time to seconds with millisecond precision
        float app_elapsed_time_tmp = elapsed_time.count() / 1000.0f;

        g_dt = app_elapsed_time_tmp - app_elapsed_time_prv;

        // Truncate the number to three decimal places
        g_app_elapsed_time = (std::floor(app_elapsed_time_tmp * 1000.0f) / 1000.0f);
        app_elapsed_time_prv = g_app_elapsed_time;
    }
}

/********************************************************************************
 * Function: shutdown
 * Description: Shutdown time calculation.
 ********************************************************************************/
void Time::shutdown(void)
{
    
}
