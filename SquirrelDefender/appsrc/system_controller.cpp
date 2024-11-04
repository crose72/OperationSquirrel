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
#include "datalog.h"
#include "vehicle_controller.h"
#include "video_IO.h"
#include "detect_target.h"

#ifdef ENABLE_CV

#include "follow_target.h"

#endif // ENABLE_CV

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/
DebugTerm SysStat("");
bool systems_initialized;
SYSTEM_STATE system_state;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/
int system_state_machine(void);
void led_system_indicators(void);

/********************************************************************************
 * Function: system_state_machine
 * Description: Determine system state,.
 ********************************************************************************/
int system_state_machine(void)
{
    // Initialize system status on startup
    if (first_loop_after_start)
    {
        system_state = SYSTEM_STATE::DEFAULT;
    }
    else
    {
        bool mav_type_is_quad = (mav_veh_type == MAV_TYPE_QUADROTOR && mav_veh_autopilot_type == MAV_AUTOPILOT_ARDUPILOTMEGA);
        bool prearm_checks = false;

#ifdef JETSON_B01

        prearm_checks = ((mav_veh_sys_stat_onbrd_cntrl_snsrs_present & MAV_SYS_STATUS_PREARM_CHECK) != 0 && valid_image_rcvd);

#elif WSL

        prearm_checks = ((mav_veh_sys_stat_onbrd_cntrl_snsrs_present & MAV_SYS_STATUS_PREARM_CHECK) != 0);

#endif // JETSON_B01
       // Switch case determines how we transition from one state to another
        switch (system_state)
        {
        // Default state is the first state, nothing is initialialized, no systems are active
        case SYSTEM_STATE::DEFAULT:
            if (systems_initialized)
            {
                system_state = SYSTEM_STATE::INIT;
            }
            break;
        // After video, inference, SLAM, and other systems have successfully initialized we are in the init state
        case SYSTEM_STATE::INIT:
            if (prearm_checks)
            {
                system_state = SYSTEM_STATE::PRE_ARM_GOOD;
            }
            break;
        // Pre arm good means that the data is from a drone and pre arm checks are good
        case SYSTEM_STATE::PRE_ARM_GOOD:
            if (mav_type_is_quad && mav_veh_state == MAV_STATE_STANDBY)
            {
                system_state = SYSTEM_STATE::STANDBY;
            }
            else if (mav_veh_rel_alt > 1000 && mav_type_is_quad && mav_veh_state == MAV_STATE_ACTIVE)
            {
                system_state = SYSTEM_STATE::IN_FLIGHT_GOOD;
            }
            break;
        // Standby means we are ready to takeoff
        case SYSTEM_STATE::STANDBY:
            if ((mav_veh_rel_alt > 1000 || mav_veh_rngfdr_current_distance > 100) && mav_veh_state == MAV_STATE_ACTIVE)
            {
                system_state = SYSTEM_STATE::IN_FLIGHT_GOOD;
            }
            break;
        // In flight good means the vehicle is in the air and has no system failures
        case SYSTEM_STATE::IN_FLIGHT_GOOD:

            break;
        // In flight good means the vehicle is in the air with some system failures (e.g. video feed stopped)
        case SYSTEM_STATE::IN_FLIGHT_ERROR:

            break;
        }
    }

    return 0;
}

#ifdef JETSON_B01

/********************************************************************************
 * Function: led_system_indicators
 * Description: Control external leds to describe the system state.
 ********************************************************************************/
void led_system_indicators(void)
{
    if (system_state == SYSTEM_STATE::DEFAULT ||
        system_state == SYSTEM_STATE::INIT ||
        system_state == SYSTEM_STATE::PRE_ARM_GOOD ||
        system_state == SYSTEM_STATE::IN_FLIGHT_GOOD)
    {
        StatusIndicators::status_good();
    }
    else
    {
        StatusIndicators::status_good();
    }
}

#endif // JETSON_B01

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
 * Function: init
 * Description: Return 0 if all system init tasks have successfully completed.
 ********************************************************************************/
int SystemController::init(void)
{
    systems_initialized = false;

#ifdef ENABLE_CV

    StatusIndicators::init();
    StatusIndicators::status_initializing();

    if (!Video::init() ||
        !Detection::init() ||
        !Track::init() ||
        !Localize::init() ||
        !Follow::init() ||
        !VehicleController::init())
    {
#ifdef JETSON_B01

        StatusIndicators::status_bad_blink();

#endif // JETSON_B01
        return 1;
    }

    StatusIndicators::status_initializing();

    if (!MavMsg::init() ||
        !DataLogger::init() ||
        !VehicleController::init())
    {
#ifdef JETSON_B01

        StatusIndicators::status_bad_blink();

#endif // JETSON_B01
        return 1;
    }

#else // WSL

    if (!MavMsg::init() ||
        !DataLogger::init() ||
        !VehicleController::init())
    {
        return 1;
    }

#endif // ENABLE_CV

    systems_initialized = true;

    return 0;
}

/********************************************************************************
 * Function: loop
 * Description: Main loop for functions that monitor and control system states.
 ********************************************************************************/
void SystemController::loop(void)
{
    system_state_machine();
    app_first_init();

#ifdef JETSON_B01

    led_system_indicators();
    StatusIndicators::loop();

#endif // JETSON_B01
}

/********************************************************************************
 * Function: shutdown
 * Description: All shutdown functions are called here.
 ********************************************************************************/
void SystemController::shutdown(void)
{
    VehicleController::shutdown();
    MavMsg::shutdown();

#ifdef ENABLE_CV

    Video::shutdown();
    Detection::shutdown();

#endif // ENABLE_CV

#ifdef JETSON_B01

    StatusIndicators::status_program_complete();
    StatusIndicators::shutdown();

#endif // JETSON_B01


}
