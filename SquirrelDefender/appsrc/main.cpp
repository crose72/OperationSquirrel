/********************************************************************************
 * @file    main.cpp
 * @author  Cameron Rose
 * @date    6/7/2024
 * @brief   Main entry point for the program, contains the loop functions for 
            each software component.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include <mutex>
#include <signal.h>
#include <chrono>
#include <thread>
#include "common_inc.h"
#include "datalog.h"
#include "video_io.h"
#include "system_controller.h"
#include "mavlink_msg_handler.h"
#include "mav_utils.h"
#include "vehicle_controller.h"
#include "detect_target.h"
#include "track_target.h"
#include "localize_target.h"
#include "follow_target.h"
#include "time_calc.h"

#ifdef BLD_JETSON_B01

#include "status_io.h"
#include <jsoncpp/json/json.h> // sudo apt-get install libjsoncpp-dev THEN target_link_libraries(your_executable_name jsoncpp)

#endif // BLD_JETSON_B01

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
        Print::c_fprintf("received SIGINT\n");
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
        Print::c_fprintf("can't catch SIGINT");
    }
}

/********************************************************************************
 * Function: main
 * Description: Entry point for the program.  Runs the main loop.
 ********************************************************************************/
int main(void)
{
    attach_sig_handler();
    stop_program = false;

    if (SystemController::init() != 0)
    {
        return SystemController::init();
    }

#ifdef BLD_JETSON_B01

    while (!stop_program && !g_save_button_press)

#else

    while (!stop_program)

#endif // BLD_JETSON_B01

    {
        std::lock_guard<std::mutex> lock(mutex_main);
        TimeCalc::loop();
        SystemController::loop();
        MavMsg::loop();

#ifdef ENABLE_CV

        Video::in_loop();
        Detection::loop();
        Track::loop();
        Localize::loop();
        Follow::loop();
        Video::out_loop();

#endif // ENABLE_CV

        VehicleController::loop();
        DataLogger::loop();
    }

    SystemController::shutdown();

    return 0;
}
