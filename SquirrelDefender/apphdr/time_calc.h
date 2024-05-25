#pragma once

/********************************************************************************
 * @file    time_calc.h
 * @date    12/27/2023
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
 * Exported objects
 ********************************************************************************/
extern float app_elapsed_time;

/********************************************************************************
 * Function prototypes
 ********************************************************************************/

class TimeCalc
{
    public:
        TimeCalc();
        ~TimeCalc();

        std::chrono::time_point<std::chrono::steady_clock> app_start_time;
        std::chrono::time_point<std::chrono::steady_clock> app_end_time;
        std::chrono::time_point<std::chrono::steady_clock> loop_start_time;
        std::chrono::time_point<std::chrono::steady_clock> loop_end_time;
        std::chrono::milliseconds loop_duration;

        void calc_app_start_time(void);
        void calc_app_end_time(void);
        void calc_loop_start_time(void);
        void calc_loop_end_time(void);
        void loop_rate_controller(void);
        void calc_elapsed_time(void);

    private:
    };

#endif // TIME_CALC_H
