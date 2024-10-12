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
#include "vehicle_controller.h"
#include "follow_target.h"
#include "localize_target.h"
#include "datalog.h"
#include <mutex>
#include <signal.h>

#ifdef JETSON_B01

#include "track_target.h"
#include "jetson_IO.h"
#include "video_IO.h"
#include "detect_target.h"
#include <jsoncpp/json/json.h> // sudo apt-get install libjsoncpp-dev THEN target_link_libraries(your_executable_name jsoncpp)

#endif // JETSON_B01

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/
bool stop_program;
std::mutex mutex_main;
DebugTerm MainTerm("");

extern bool save_button_press;

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
    TimeCalc MainAppTime;

    MainAppTime.calc_app_start_time();
    attach_sig_handler();
    stop_program = false;

    if (save_button_press)
    {
        return 2;
    }

    if (SystemController::init() != 0)
    {
        StatusIndicators::status_bad_blink();
        return 1;
    }

#ifdef JETSON_B01

    while (!stop_program && !save_button_press)

#else

    while (!stop_program)

#endif // JETSON_B01

    {
        std::lock_guard<std::mutex> lock(mutex_main);
        SystemController::loop();
        MavMsg::loop();

#ifdef JETSON_B01

        Video::in_loop();
        Detection::loop();
        Track::loop();
        Localize::loop();
        Track::loop();
        Follow::loop();
        VehicleController::loop();
        Video::out_loop();
        StatusIndicators::loop();

#elif WSL

        VehicleController::loop();

#endif // JETSON_B01

        app_first_init();

        DataLogger::loop();

        MainAppTime.calc_elapsed_time();
        MainAppTime.loop_rate_controller();
        MainAppTime.calc_loop_start_time();
    }

    SystemController::shutdown();
    StatusIndicators::status_program_complete();
    StatusIndicators::gpio_shutdown();

    return 0;
}
