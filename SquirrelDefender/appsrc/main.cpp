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

#ifdef USE_JETSON
	#include "video_IO.h"
	#include "object_detection.h"
	#include <jsoncpp/json/json.h> // sudo apt-get install libjsoncpp-dev THEN target_link_libraries(your_executable_name jsoncpp)
#endif // USE_JETSON

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
TimeCalc MainAppTime;
bool signal_recieved = false;
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
	if( signo == SIGINT )
	{
		PrintPass::c_printf("received SIGINT\n");
		signal_recieved = true;
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
        PrintPass::c_printf("can't catch SIGINT\n");
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
    
    if(SystemController::system_init() != 0)
    {
        return SystemController::system_init();
    }

    while (!signal_recieved) 
	{
        std::lock_guard<std::mutex> lock(mutex);
		MainAppTime.calc_elapsed_time();
		MavMsg::mav_comm_loop();
		
		#ifdef USE_JETSON

			Video::video_proc_loop();
			Detection::detection_loop();
			Follow::follow_target_loop();

		#elif USE_WSL

			test_flight();

		#endif // USE_JETSON	

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