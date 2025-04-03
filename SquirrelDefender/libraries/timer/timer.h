#pragma once

/********************************************************************************
 * @file    timer.h
 * @date    1/22/2025
 ********************************************************************************/
#ifndef TIMER_H
#define TIMER_H

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
        Timer(std::chrono::milliseconds loop_rate_desired);
        ~Timer();

        std::chrono::time_point<std::chrono::steady_clock> loop_start_time;
        std::chrono::time_point<std::chrono::steady_clock> loop_end_time;
        std::chrono::milliseconds loop_duration;

        void start_time(void);
        void end_time(void);
        void wait(void);

    private:
        std::chrono::milliseconds loop_rate_desired;
 };

#endif // TIMER_H
