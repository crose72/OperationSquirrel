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

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/
bool systems_initialized;
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
            if (systems_initialized)
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

    StatusIndicators::init();

#endif // BLD_JETSON_B01
}

/********************************************************************************
 * Function: led_init_blink
 * Description: Specific led sequence to indicate system initializing.
 ********************************************************************************/
void led_init_blink(void)
{
#ifdef BLD_JETSON_B01

    StatusIndicators::status_initializing();

#endif // BLD_JETSON_B01
}

/********************************************************************************
 * Function: led_bad_blink
 * Description: Control external leds to describe the system state.
 ********************************************************************************/
void led_bad_blink(void)
{
#ifdef BLD_JETSON_B01

        StatusIndicators::status_bad_blink();

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
        StatusIndicators::status_good();
    }
    else
    {
        StatusIndicators::status_good();
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
int SystemController::init(void)
{
    systems_initialized = false;
    
#ifdef BLD_JETSON_B01

    if (g_save_button_press)
    {
        return 2;
    }

#endif // BLD_JETSON_B01

#ifdef ENABLE_CV

    led_init();
    led_init_blink();
    
    if (!Video::init() ||
        !Detection::init() ||
        !Track::init() ||
        !Localize::init() ||
        !Follow::init())
    {
        led_bad_blink();
        return 1;
    }

    led_init_blink();
    
    if (!MavMsg::init() ||
        !DataLogger::init() ||
        !VehicleController::init() ||
        !Time::init())
    {
        led_bad_blink();
        return 1;
    }

#else // WSL is the only platform that doesn't support computer vision at the moment

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
    led_system_indicators();

#ifdef BLD_JETSON_B01

    StatusIndicators::loop();

#endif // BLD_JETSON_B01
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

#ifdef BLD_JETSON_B01

    StatusIndicators::status_program_complete();
    StatusIndicators::shutdown();

#endif // BLD_JETSON_B01


}
