/********************************************************************************
 * @file    main.cpp
 * @author  Cameron Rose
 * @date    12/27/2023
 * @brief   Main entry point for the program
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "mavlink_msg_handler.h"
#include "mavlink_cmd_handler.h"
#include "scheduler.h"
#include "datalog.h"
#include "time_calc.h"

// Test flights
// #include "sim_flight_test_1_GPSWaypoints.h"
// #include "sim_flight_test_2_AttitudeControl.h"
// #include "sim_flight_test_3_VelocityControl.h"
#include "sim_flight_test_4_VelocityControl.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/
MavCmd mav;
SerialPort serial;
TimeCalc tm;
MavMsg mavmsg;
/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: main
 * Description: Main point of entry for the program.
 ********************************************************************************/
int main() 
{
    // Code here is to be run once at the start of the program
    tm.app_start_time();
    serial.open_serial_port();
    mavmsg.set_message_rates();
    mavmsg.request_messages();
    mav.takeoff_sequence((float)6);

    while (!stopProgram) 
	{
        tm.elapsed_time();
        tm.loop_start_time();
        mavmsg.parse_serial_data();
        test_flight();

        // logData();

        if (firstLoopAfterStartup == true)
        {
            firstLoopAfterStartup = false;
        }
        tm.loop_end_time();
        tm.loop_rate_controller();
    }

    return 0;
}