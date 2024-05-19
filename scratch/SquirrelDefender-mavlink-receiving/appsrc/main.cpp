/********************************************************************************
 * @file    ${file_name}
 * @author  ${user}
 * @date    ${date}
 * @brief   
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "global_objects.h"
#include "global_calibrations.h"
#include "serial_port_handler.h"
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
constexpr int interval_ms = 25;
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
    initialize();
    startTask_25ms();
        // Get the current time
    auto start_time = std::chrono::steady_clock::now();
    float desired_width;

    while (!stopProgram) 
	{
        std::lock_guard<std::mutex> lock(mutex);
        calcElapsedTime();
        parse_serial_data();
        test_flight();
        // logData();

        // Calculate elapsed time since the start of the loop
		auto end_time = std::chrono::steady_clock::now();
		auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
		

		if (firstLoopAfterStartup == true)
		{
		    firstLoopAfterStartup = false;
		}

		if (elapsed_time < interval_ms)
		{
		    auto sleep_duration = std::chrono::milliseconds(interval_ms - elapsed_time);
		    std::this_thread::sleep_for(sleep_duration);
		}

		// Update start time for the next iteration
		start_time = std::chrono::steady_clock::now();
    }

    stopTask_25ms();

    return 0;
}

/********************************************************************************
 * Function: initialize
 * Description: Any code that needs to be run once at the start of the program.
 ********************************************************************************/
void initialize(void)
{
    // Code here is to be run once at the start of the program
    calcStartTimeMS();
    setupTask_25ms();

    // Setup serial communication
    open_serial_port();

    // Set rates and request mavlink data from the flight controller
    set_message_rates();
    request_messages();

    // Startup commands for drone
    takeoff_sequence((float)6);
}

/********************************************************************************
 * Function: task_25ms
 * Description: Function to handle timer interrupt at 25ms (40Hz).  This periodic
 *              function runs the main code at the specified rate.
 ********************************************************************************/
void task_25ms(int sig, siginfo_t* si, void* uc)
{
    //
}