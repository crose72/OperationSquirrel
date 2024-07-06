/********************************************************************************
 * @file    target_tracking.cpp
 * @author  Cameron Rose
 * @date    6/7/2024
 * @brief   Main source file where initializations, loops, and shutdown
 * 			sequences are executed.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "system_controller.h"
#include "mavlink_msg_handler.h"
#include "mavlink_cmd_handler.h"
#include "follow_target.h"
#include "datalog.h"
#include <mutex>
#include <signal.h>

#ifdef USE_JETSON

#include "video_IO.h"
#include "object_detection.h"
#include <jsoncpp/json/json.h> // sudo apt-get install libjsoncpp-dev THEN target_link_libraries(your_executable_name jsoncpp)

#endif // USE_JETSON

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/
TimeCalc MainAppTime;
bool stop_program;
std::mutex mutex;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: sig_handler
 * Description: Stop the program if Crl+C is entered in the terminal.
 ********************************************************************************/
void sig_handler(int signo)
{
    if (signo == SIGINT)
    {
        stop_program = true;
        PrintPass::c_fprintf("received SIGINT\n");
    }
}

/********************************************************************************
 * Function: attach_sig_handler
 * Description: Attach sig handler to enable program termination by Crl+C.
 ********************************************************************************/
void attach_sig_handler(void)
{
    if (signal(SIGINT, sig_handler) == SIG_ERR)
    {
        PrintPass::c_fprintf("can't catch SIGINT");
    }
}

/********************************************************************************
 * Function: app_first_init
 * Description: Updates variable for rest of program to know that the first loop
 * 				is over.
 ********************************************************************************/
void app_first_init(void)
{
    if (first_loop_after_start == true)
    {
        first_loop_after_start = false;
    }
}

/********************************************************************************
 * Function: main
 * Description: Entry point for the program.  Runs the main loop.
 ********************************************************************************/
int main(void)
{
    MainAppTime.calc_app_start_time();
    attach_sig_handler();
    stop_program = false;

    if (SystemController::system_init() != 0)
    {
        return 1;
    }

    while (!stop_program)
    {
        std::lock_guard<std::mutex> lock(mutex);
        MainAppTime.calc_elapsed_time();
        SystemController::system_control_loop();
        MavMsg::mav_comm_loop();

#ifdef USE_JETSON

        Video::video_proc_loop();
        Detection::detection_loop();
        VehicleController::vehicle_control_loop();
        Video::video_output_loop();
        StatusIndicators::io_loop();

#endif // USE_JETSON

        DataLogger::data_log_loop();
        app_first_init();

        MainAppTime.loop_rate_controller();
        MainAppTime.calc_loop_start_time();
    }

#ifdef USE_JETSON

    Video::shutdown();
    Detection::shutdown();

#endif // USE_JETSON

    MavMsg::mav_comm_shutdown();

    return 0;
}