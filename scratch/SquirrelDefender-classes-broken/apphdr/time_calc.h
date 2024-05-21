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
extern float elapsedTimeSeconds;

/********************************************************************************
 * Function prototypes
 ********************************************************************************/

class TimeCalc
{
    public:

        TimeCalc();
        ~TimeCalc();

        void app_start_time(void);
        void app_end_time(void);
        void loop_start_time(void);
        void loop_end_time(void);
        void loop_rate_controller(void);
        void elapsed_time(void);


    private:

        std::chrono::_V2::system_clock::time_point appStartTime;
        std::chrono::_V2::system_clock::time_point appEndTime;
        std::chrono::_V2::system_clock::time_point loopStartTime;
        std::chrono::_V2::system_clock::time_point loopEndTime;
        std::chrono::milliseconds loopDuration;
    
};


#endif // TIME_CALC_H