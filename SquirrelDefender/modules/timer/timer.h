#pragma once

/********************************************************************************
 * @file    timer.h
 * @date    6/7/2023
 ********************************************************************************/
#ifndef TIME_CALC_H
#define TIME_CALC_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include <chrono>
#include <thread>
#include <cmath>

/********************************************************************************
 * Imported objects
 ********************************************************************************/

/********************************************************************************
 * Exported objects
 ********************************************************************************/

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/

class Timer
{
    public:
        Timer();
        ~Timer();

        std::chrono::time_point<std::chrono::steady_clock> loop_start_time;
        std::chrono::time_point<std::chrono::steady_clock> loop_end_time;
        std::chrono::milliseconds loop_duration;

        void calc_loop_start_time(void);
        void calc_loop_end_time(void);
        void loop_rate_controller(void);
        void calc_elapsed_time(void);

    private:
    };

#endif // TIME_CALC_H
