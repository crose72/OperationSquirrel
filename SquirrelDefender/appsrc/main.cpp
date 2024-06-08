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
#include "mavlink_msg_handler.h"
#include "mavlink_cmd_handler.h"
#include "follow_target.h"
#include "datalog.h"
#include "time_calc.h"
#include <mutex>

#ifdef USE_JETSON
	#include "video_IO.h"
	#include "object_detection.h"
	#include <jsoncpp/json/json.h> //sudo apt-get install libjsoncpp-dev THEN target_link_libraries(your_executable_name jsoncpp)
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
bool signal_recieved = false;
std::mutex mutex;

#ifdef USE_JETSON
	int argc;
	char** argv;
	commandLine cmdLine(0, nullptr);
#endif // USE_JETSON

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: command_line_inputs
 * Description: Get command line inputs.
 ********************************************************************************/
#ifdef USE_JETSON

int command_line_inputs(void)
{
	commandLine cmdLine(argc, argv);

	if( cmdLine.GetFlag("help") )
	{
		return Detection::print_usage();
	}
	
	return 1;
}

#endif // USE_JETSON

/********************************************************************************
 * Function: sig_handler
 * Description: Stop the program if Crl+C is entered in the terminal.
 ********************************************************************************/
void sig_handler(int signo)
{
	if( signo == SIGINT )
	{
		printf("received SIGINT\n");
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
        printf("can't catch SIGINT\n");
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
	TimeCalc Time;

	Time.calc_app_start_time();
    
	#ifdef USE_JETSON

		command_line_inputs();
		Video::initialize_video_streams(cmdLine, ARG_POSITION(0));
		Detection::initialize_detection_network();
		
	#endif // USE_JETSON

	attach_sig_handler();
	MavMsg::start_mav_comm();
	MavMsg::message_subscriptions();
	MavCmd::set_mode_GUIDED();
	MavCmd::arm_vehicle();
	MavCmd::takeoff_GPS_long((float)2.0);

    while (!signal_recieved) 
	{
        std::lock_guard<std::mutex> lock(mutex);
		Time.calc_elapsed_time();
		MavMsg::parse_mav_msgs();
		
		#ifdef USE_JETSON

			Video::video_input_loop();
			Detection::detection_loop();
			Follow::follow_target_loop();
			Video::video_output_loop();

		#elif USE_WSL

			test_flight();

		#endif // USE_JETSON	

        Time.loop_rate_controller();
		app_first_init();
		Time.calc_loop_start_time();
    }

	#ifdef USE_JETSON

		Video::shutdown();
		Detection::shutdown();

	#endif // USE_JETSON

    MavMsg::stop_mav_comm();

    return 0;
}