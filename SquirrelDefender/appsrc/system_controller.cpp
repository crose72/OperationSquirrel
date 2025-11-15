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
#include "scheduler.h"
#include "mav_data_hub.h"
#include "mav_utils.h"
#include "datalog.h"
#include "video_io.h"
#include "status_io.h"
#include "vehicle_controller.h"
#include "detect_target.h"
#include "track_target.h"
#include "localize_target.h"
#include "path_planner.h"
#include "path_planner.h"
#include "time_calc.h"

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
uint32_t g_mav_veh_custom_mode_prv;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/
int system_state_machine(void);
void led_system_indicators(void);
void dtrmn_program_stop_cond(void);

/********************************************************************************
 * Function: system_state_machine
 * Description: Determine system state,.
 ********************************************************************************/
void dtrmn_program_stop_cond(void)
{
    if (g_mav_mode_custom == (uint32_t)9 && g_mav_veh_custom_mode_prv != (uint32_t)9)
    {
        g_ctrl_land_override = true;
    }
    else
    {
        g_ctrl_land_override = false;
    }

    if (g_system_state == SystemState::IN_FLIGHT_GOOD && g_ctrl_land_override)
    {
        g_app_stop = true;
    }

    if (!g_cam0_img_valid && g_app_use_video_playback && g_system_state == SystemState::IN_FLIGHT_GOOD)
    {
        g_app_stop = true;
    }

    if (g_video_end)
    {
        g_app_stop = true;
    }

    g_mav_veh_custom_mode_prv = g_mav_mode_custom;
}

/********************************************************************************
 * Function: system_state_machine
 * Description: Determine system state,.
 ********************************************************************************/
int system_state_machine(void)
{
    // Initialize system status on startup
    if (g_app_first_loop)
    {
        g_system_state = SystemState::DEFAULT;
    }
    else
    {
        bool mav_type_is_quad = (g_mav_type == MAV_TYPE_QUADROTOR && g_mav_autopilot_type == MAV_AUTOPILOT_ARDUPILOTMEGA);
        bool prearm_checks = false;

#ifdef ENABLE_CV

        prearm_checks = ((g_mav_sys_sensors_present & MAV_SYS_STATUS_PREARM_CHECK) != 0 && g_cam0_img_valid);

#else

        prearm_checks = ((g_mav_sys_sensors_present & MAV_SYS_STATUS_PREARM_CHECK) != 0);

#endif // ENABLE_CV

        // Switch case determines how we transition from one state to another
        switch (g_system_state)
        {
        // Default state is the first state, nothing is initialialized, no systems are active
        case SystemState::DEFAULT:
            if (g_system_init)
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
            if (mav_type_is_quad && g_mav_state == MAV_STATE_STANDBY)
            {
                g_system_state = SystemState::STANDBY;
            }
            else if (g_mav_gps_alt_rel > 1000 && mav_type_is_quad && g_mav_state == MAV_STATE_ACTIVE)
            {
                g_system_state = SystemState::IN_FLIGHT_GOOD;
            }
            break;
        // Standby means we are ready to takeoff
        case SystemState::STANDBY:
            if ((g_mav_gps_alt_rel > 1000 || g_mav_rngfndr_dist_m > 100) && g_mav_state == MAV_STATE_ACTIVE)
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
 * Function: led_system_indicators
 * Description: Control external leds to describe the system state.
 ********************************************************************************/
void led_system_indicators(void)
{
    if (g_system_state == SystemState::DEFAULT ||
        g_system_state == SystemState::INIT ||
        g_system_state == SystemState::PRE_ARM_GOOD ||
        g_system_state == SystemState::IN_FLIGHT_GOOD)
    {
        /* TODO: blinking green LED */
    }
    else
    {
        /* TODO: blinking red LED */
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
bool SystemController::init(void)
{
    g_system_init = false;
    g_mav_veh_custom_mode_prv = 0;

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
    dtrmn_program_stop_cond();
}

/********************************************************************************
 * Function: shutdown
 * Description: All shutdown functions are called here.
 ********************************************************************************/
void SystemController::shutdown(void)
{
}
