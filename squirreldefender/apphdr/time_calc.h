/********************************************************************************
 * @file    time_calc.h
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Application-wide timing utilities, maintaining frame time,
 *          delta time, and epoch timestamps. Used by all modules for
 *          synchronized timing.
 ********************************************************************************/
#ifndef TIME_CALC_H
#define TIME_CALC_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include <cstdint>

/********************************************************************************
 * Exported objects
 ********************************************************************************/
extern float g_app_time_s;
extern uint64_t g_app_time_ns;
extern float g_app_dt;
extern bool g_app_first_loop;
extern uint64_t g_app_epoch_ns;

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
