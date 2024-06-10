/********************************************************************************
 * @file    system_controller.cpp
 * @author  Cameron Rose
 * @date    6/7/2023
 * @brief   Control the position, velocity, and acceleration of the drone by 
 *          sending the following MAVLINK message to the drone.  Control the
 *          vector position, velocity, acceleration, and yaw/yaw rate.
 * 
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "system_controller.h"
#include "mavlink_msg_handler.h"
#include "mavlink_cmd_handler.h"

#ifdef USE_JETSON
    #include "video_IO.h"
    #include "object_detection.h"
    #include <jsoncpp/json/json.h> //sudo apt-get install libjsoncpp-dev THEN target_link_libraries(your_executable_name jsoncpp)
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

#ifdef USE_JETSON

/********************************************************************************
 * Function: command_line_inputs
 * Description: Get command line inputs.
 ********************************************************************************/
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
 * Function: SystemController
 * Description: Class constructor
 ********************************************************************************/
SystemController::SystemController(void){}

/********************************************************************************
 * Function: ~SystemController
 * Description: Class destructor
 ********************************************************************************/
SystemController::~SystemController(void){}

/********************************************************************************
 * Function: system_init
 * Description: All init functions are called here.
 ********************************************************************************/
bool SystemController::system_init(void)
{
    #ifdef USE_JETSON

        command_line_inputs();

        if (!Video::initialize_video_streams(cmdLine, ARG_POSITION(0)) || 
            !Detection::initialize_detection_network())
        {
            return false;
        }

    #endif // USE_JETSON

    MavMsg::start_mav_comm();
    /*if (!MavMsg::start_mav_comm()) 
    {
        std::cerr << "Failed to start MAVLink communication" << std::endl;
        return false;
    }*/

    MavMsg::message_subscriptions();
    MavCmd::set_mode_GUIDED();
    MavCmd::arm_vehicle();
    MavCmd::takeoff_GPS_long((float)2.0);
}

/********************************************************************************
 * Function: system_shutdown
 * Description: All shutdown functions are called here.
 ********************************************************************************/
void SystemController::system_shutdown(void)
{
    #ifdef USE_JETSON

        Video::shutdown();
        Detection::shutdown();

    #endif // USE_JETSON

    MavMsg::stop_mav_comm();
}

/********************************************************************************
 * Function: system_state
 * Description: Determine system state,.
 ********************************************************************************/
int SystemController::system_state(void)
{

}