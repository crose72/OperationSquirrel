/********************************************************************************
 * @file    system_controller.cpp
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   High level state machine monitoring the state of flight, video 
 *          readiness and other signals to determine if the vehicle is ready to
 *          proceed with a course of action (e.g. ready to takeoff, error need to
 *          land).
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "system_controller.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/
SystemState g_system_state;

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
    if (g_first_loop_after_start)
    {
        g_system_state = SystemState::DEFAULT;
    }
    else
    {
        bool mav_type_is_quad = (g_mav_veh_type == MAV_TYPE_QUADROTOR && g_mav_veh_autopilot_type == MAV_AUTOPILOT_ARDUPILOTMEGA);
        bool prearm_checks = false;

#ifdef ENABLE_CV

        prearm_checks = ((g_mav_veh_sys_stat_onbrd_cntrl_snsrs_present & MAV_SYS_STATUS_PREARM_CHECK) != 0 && g_valid_image_rcvd);

#else

        prearm_checks = ((g_mav_veh_sys_stat_onbrd_cntrl_snsrs_present & MAV_SYS_STATUS_PREARM_CHECK) != 0);

#endif // ENABLE_CV

       // Switch case determines how we transition from one state to another
        switch (g_system_state)
        {
        // Default state is the first state, nothing is initialialized, no systems are active
        case SystemState::DEFAULT:
            if (controller_initialiazed)
            {
                g_system_state = SystemState::INIT;
            }
            break;
        // After video, inference, SLAM, and other systems have successfully initialized we are in the init state
        case SystemState::INIT:
            if (prearm_checks)
            {
                g_system_state = SystemState::PRE_ARM_GOOD;
            }
            break;
        // Pre arm good means that the data is from a drone and pre arm checks are good
        case SystemState::PRE_ARM_GOOD:
            if (mav_type_is_quad && g_mav_veh_state == MAV_STATE_STANDBY)
            {
                g_system_state = SystemState::STANDBY;
            }
            else if (g_mav_veh_rel_alt > 1000 && mav_type_is_quad && g_mav_veh_state == MAV_STATE_ACTIVE)
            {
                g_system_state = SystemState::IN_FLIGHT_GOOD;
            }
            break;
        // Standby means we are ready to takeoff
        case SystemState::STANDBY:
            if ((g_mav_veh_rel_alt > 1000 || g_mav_veh_rngfdr_current_distance > 100) && g_mav_veh_state == MAV_STATE_ACTIVE)
            {
                g_system_state = SystemState::IN_FLIGHT_GOOD;
            }
            break;
        // In flight good means the vehicle is in the air and has no system failures
        case SystemState::IN_FLIGHT_GOOD:

            break;
        // In flight good means the vehicle is in the air with some system failures (e.g. video feed stopped)
        case SystemState::IN_FLIGHT_ERROR:

            break;
        }
    }

    return 0;
}

/********************************************************************************
 * Function: led_init
 * Description: Initialize LEDs
 ********************************************************************************/
void led_init(void)
{
#ifdef BLD_JETSON_B01

    StatusIO::init();

#endif // BLD_JETSON_B01
}

/********************************************************************************
 * Function: led_init_blink
 * Description: Specific led sequence to indicate system initializing.
 ********************************************************************************/
void led_init_blink(void)
{
#ifdef BLD_JETSON_B01

    StatusIO::status_initializing();

#endif // BLD_JETSON_B01
}

/********************************************************************************
 * Function: led_bad_blink
 * Description: Control external leds to describe the system state.
 ********************************************************************************/
void led_bad_blink(void)
{
#ifdef BLD_JETSON_B01

        StatusIO::status_bad_blink();

#endif // BLD_JETSON_B01
}

/********************************************************************************
 * Function: led_system_indicators
 * Description: Control external leds to describe the system state.
 ********************************************************************************/
void led_system_indicators(void)
{
#ifdef BLD_JETSON_B01

    if (g_system_state == SystemState::DEFAULT ||
        g_system_state == SystemState::INIT ||
        g_system_state == SystemState::PRE_ARM_GOOD ||
        g_system_state == SystemState::IN_FLIGHT_GOOD)
    {
        StatusIO::status_good();
    }
    else
    {
        StatusIO::status_good();
    }

#endif // BLD_JETSON_B01
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
bool SystemController::init(void)
{
    controller_initialiazed = false;

#ifdef BLD_JETSON_B01

    if (g_save_button_press)
    {
        return false;
    }

#endif // BLD_JETSON_B01

    return true;
}

/********************************************************************************
 * Function: loop
 * Description: Main loop for functions that monitor and control system states.
 ********************************************************************************/
void SystemController::loop(void)
{
    system_state_machine();

#ifdef BLD_JETSON_B01

    led_system_indicators();
    StatusIO::loop();

#endif // BLD_JETSON_B01
}

/********************************************************************************
 * Function: shutdown
 * Description: All shutdown functions are called here.
 ********************************************************************************/
void SystemController::shutdown(void)
{

}
