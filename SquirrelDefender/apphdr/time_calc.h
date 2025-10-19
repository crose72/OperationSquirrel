/********************************************************************************
 * @file    time_calc.h
 * @date    1/22/2025
 ********************************************************************************/
#ifndef TIME_CALC_H
#define TIME_CALC_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include <chrono>
#include <thread>
#include <cmath>

/********************************************************************************
 * Imported objects
 ********************************************************************************/

/********************************************************************************
 * Exported objects
 ********************************************************************************/
extern float g_app_elapsed_time;
extern uint64_t g_app_elapsed_time_ns;
extern float g_dt;
extern bool g_first_loop_after_start;
extern uint64_t g_epoch_ns;

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/

class Time
{
public:
    Time();
    ~Time();

    static bool init(void);
    static void loop(void);
    static void shutdown(void);

private:
};

#endif // TIME_CALC_H
