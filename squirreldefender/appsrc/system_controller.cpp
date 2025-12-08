/********************************************************************************
 * @file    system_controller.cpp
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   High-level system state machine.
 *
 *          This module monitors MAVLink state, vehicle readiness, video feed
 *          status, and other signals to determine whether the vehicle is ready
 *          to proceed with actions such as takeoff or landing. It can also
 *          trigger program termination when sensors fail or when flight ends.
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
#include "target_detection.h"
#include "target_tracking.h"
#include "target_localization.h"
#include "velocity_controller.h"
#include "time_calc.h"
#include "param_reader.h"

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
uint32_t min_flight_alt_mm;
uint16_t min_flight_alt_cm;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/
int system_state_machine(void);
void led_system_indicators(void);
void dtrmn_program_stop_cond(void);
void get_system_params(void);

/********************************************************************************
 * Function: dtrmn_program_stop_cond
 * Description:
 *      Determine if the program should stop due to:
 *        - Landing override signal
 *        - Lost video frame during playback mode
 *        - End-of-video playback
 ********************************************************************************/
void get_system_params(void)
{
    ParamReader system_params("../params.json");

    min_flight_alt_mm = system_params.get_uint32_param("system_params.min_flight_alt_mm");
    min_flight_alt_cm = system_params.get_uint16_param("system_params.min_flight_alt_cm");
}

/********************************************************************************
 * Function: dtrmn_program_stop_cond
 * Description:
 *      Determine if the program should stop due to:
 *        - Landing override signal
 *        - Lost video frame during playback mode
 *        - End-of-video playback
 ********************************************************************************/
void dtrmn_program_stop_cond(void)
{
    // Checking if the pilot sent a command to change mode to LAND
    if (g_mav_mode_custom == (uint32_t)ArduPilotMode::LAND && g_mav_veh_custom_mode_prv != (uint32_t)ArduPilotMode::LAND)
    {
        g_ctrl_land_override = true;
    }
    else
    {
        g_ctrl_land_override = false;
    }

    // Manual mode change to LAND from the user with the transmitter should end the program
    if (g_system_state == SystemState::IN_FLIGHT_GOOD && g_ctrl_land_override)
    {
        g_app_stop = true;
    }

    // Frames stop coming while in the air
    if (!g_cam0_img_valid && g_system_state == SystemState::IN_FLIGHT_GOOD)
    {
        g_app_stop = true;
    }

    // End-of-video signal
    if (g_video_end)
    {
        g_app_stop = true;
    }

    g_mav_veh_custom_mode_prv = g_mav_mode_custom;
}

/********************************************************************************
 * Function: system_state_machine
 * Description:
 *      Update the high-level system state machine using MAVLink telemetry,
 *      sensor readiness, and flight indicators.
 ********************************************************************************/
int system_state_machine(void)
{
    // First loop after program start
    if (g_app_first_loop)
    {
        g_system_state = SystemState::DEFAULT;
    }
    else
    {
        bool mav_type_is_quad = (g_mav_type == MAV_TYPE_QUADROTOR && g_mav_autopilot_type == MAV_AUTOPILOT_ARDUPILOTMEGA);
        bool prearm_checks = false;

#ifdef ENABLE_CV

        // Prearm requires ArduPilot checks + valid camera image
        prearm_checks = ((g_mav_sys_sensors_present & MAV_SYS_STATUS_PREARM_CHECK) != 0 && g_cam0_img_valid);

#else

        // Prearm requires ArduPilot checks only during simulation
        prearm_checks = ((g_mav_sys_sensors_present & MAV_SYS_STATUS_PREARM_CHECK) != 0);

#endif // ENABLE_CV

        // Switch case determines how we transition from one state to another
        switch (g_system_state)
        {
        // Default state is the first state, nothing is initialialized, no systems are active
        case SystemState::DEFAULT:
            /* DEFAULT → INIT
             * System has started, waiting for all subsystems to initialize.
             */
            if (g_system_init)
            {
                g_system_state = SystemState::INIT;
            }
            break;
        // After video, inference, SLAM, and other systems have successfully initialized we are in the init state
        case SystemState::INIT:
            /* INIT → PRE_ARM_GOOD
             * All modules initialized successfully AND vehicle prearm checks passed.
             */

            if (prearm_checks)
            {
                g_system_state = SystemState::PRE_ARM_GOOD;
            }
            break;
        // Pre arm good means that the data is from a drone and pre arm checks are good
        case SystemState::PRE_ARM_GOOD:
            /* PRE_ARM_GOOD → STANDBY
             * Vehicle is on the ground and in STANDBY state.
             * Vehicle may also be in the air during MAV_STATE_STANDBY
             */
            if (mav_type_is_quad && g_mav_state == MAV_STATE_STANDBY)
            {
                g_system_state = SystemState::STANDBY;
            }
            // PRE_ARM_GOOD → IN_FLIGHT_GOOD (if already airborne)
            else if (g_mav_gps_alt_rel > min_flight_alt_mm && mav_type_is_quad && g_mav_state == MAV_STATE_ACTIVE)
            {
                g_system_state = SystemState::IN_FLIGHT_GOOD;
            }
            break;
        // Standby means we are ready to takeoff
        case SystemState::STANDBY:
            if ((g_mav_gps_alt_rel > min_flight_alt_mm || g_mav_rngfndr_dist_cm > min_flight_alt_cm) && g_mav_state == MAV_STATE_ACTIVE)
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
    get_system_params();

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
