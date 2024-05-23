/********************************************************************************
 * @file    time_calc.h
 * @author  Cameron Rose
 * @date    12/27/2023
 ********************************************************************************/
#ifndef TIME_CALC_H
#define TIME_CALC_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"

/********************************************************************************
 * Imported objects
 ********************************************************************************/

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

        std::chrono::_V2::system_clock::time_point app_start_time;
        std::chrono::_V2::system_clock::time_point app_end_time;
        std::chrono::_V2::system_clock::time_point loop_start_time;
        std::chrono::_V2::system_clock::time_point loop_end_time;
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