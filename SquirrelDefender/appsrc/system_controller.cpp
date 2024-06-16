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
#include "follow_target.h"
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
DebugTerm SysStat("/dev/pts/3");
#ifdef USE_JETSON

int argc;
char **argv;
commandLine cmdLine(0, nullptr);

#endif // USE_JETSON

bool systems_initialized;
SYSTEM_STATE system_state;

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

    if (cmdLine.GetFlag("help"))
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
SystemController::SystemController(void) {}

/********************************************************************************
 * Function: ~SystemController
 * Description: Class destructor
 ********************************************************************************/
SystemController::~SystemController(void) {}

/********************************************************************************
 * Function: system_init
 * Description: Return 0 if all system init tasks have successfully completed.
 ********************************************************************************/
int SystemController::system_init(void)
{
    systems_initialized = false;

#ifdef USE_JETSON

    command_line_inputs();

    if (!Video::video_init(cmdLine, ARG_POSITION(0)) ||
        !Detection::detection_net_init() ||
        !Follow::follow_target_init())
    {
        return 1;
    }

#endif // USE_JETSON

    if (!MavMsg::mav_comm_init())
    {
        return 1;
    }

    systems_initialized = true;

    return 0;
}

/********************************************************************************
 * Function: system_state_machine
 * Description: Determine system state,.
 ********************************************************************************/
int SystemController::system_state_machine(void)
{
    // Initialize system status on startup
    if (first_loop_after_start)
    {
        system_state = SYSTEM_STATE::DEFAULT;
    }
    else
    {
        // Switch case determines how we transition from one state to another
        switch (system_state)
        {
        case SYSTEM_STATE::DEFAULT:
            if (systems_initialized)
            {
                system_state = SYSTEM_STATE::INIT;
            }
            break;
        case SYSTEM_STATE::INIT:
            if ((mav_veh_sys_stat_onbrd_cntrl_snsrs_present & MAV_SYS_STATUS_PREARM_CHECK) != 0 && valid_image_rcvd)
            {
                system_state = SYSTEM_STATE::PRE_ARM_GOOD;
            }
            break;
        case SYSTEM_STATE::PRE_ARM_GOOD:
            if (mav_veh_type == MAV_TYPE_QUADROTOR && mav_veh_autopilot_type == MAV_AUTOPILOT_ARDUPILOTMEGA && mav_veh_state == MAV_STATE_STANDBY)
            {
                system_state = SYSTEM_STATE::STANDBY;
            }
            break;
        case SYSTEM_STATE::STANDBY:
            if (mav_rel_alt > 120)
            {
                system_state = SYSTEM_STATE::IN_FLIGHT_GOOD;
            }
            break;
        case SYSTEM_STATE::IN_FLIGHT_GOOD:

            break;
        case SYSTEM_STATE::IN_FLIGHT_ERROR:

            break;
        }
    }

    SysStat.cpp_cout("System State: " + std::to_string(system_state));
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

    MavMsg::mav_comm_shutdown();
}
