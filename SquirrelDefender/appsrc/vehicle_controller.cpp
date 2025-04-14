/********************************************************************************
 * @file    vehicle_controller.cpp
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Send vehicle motion requests or commands based on the state of 
 *          the system and feedback loop.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "vehicle_controller.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/
bool takeoff_dbc;
bool start_follow_mode;
uint16_t takeoff_dbc_cnt;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/
void follow_mode(void);
void dtrmn_veh_control_action(void);

#ifdef ENABLE_CV

/********************************************************************************
 * Function: follow_target
 * Description: Pass control outputs from the follow algorithm to the vehicle.
 ********************************************************************************/
void follow_mode(void)
{
    float target_velocity[3] = {0.0, 0.0, 0.0};

    target_velocity[0] = g_vx_adjust;
    target_velocity[1] = g_vy_adjust;
    target_velocity[2] = g_vz_adjust;

    if (g_target_valid)
    {
        MavMotion::cmd_velocity_NED(SENDER_SYS_ID, SENDER_COMP_ID, TARGET_SYS_ID, TARGET_COMP_ID, target_velocity, g_yaw_adjust);
    }
}

#endif // ENABLE_CV

/********************************************************************************
 * Function: dtrmn_veh_control_action
 * Description: Choose vehicle action based on vehicle and system state.
 ********************************************************************************/
void dtrmn_veh_control_action(void)
{
    if (g_system_state == SystemState::INIT)
    {
        MavCmd::set_mode_guided(SENDER_SYS_ID, SENDER_COMP_ID, TARGET_SYS_ID, TARGET_COMP_ID);
    }
    else if (g_system_state == SystemState::PRE_ARM_GOOD)
    {
        MavCmd::arm_vehicle(SENDER_SYS_ID, SENDER_COMP_ID, TARGET_SYS_ID, TARGET_COMP_ID);
    }
    else if (g_system_state == SystemState::STANDBY)
    {
        MavCmd::takeoff_gps(SENDER_SYS_ID, SENDER_COMP_ID, TARGET_SYS_ID, TARGET_COMP_ID, (float)7.0);
    }
    else if (g_system_state == SystemState::IN_FLIGHT_GOOD)
    {
        /* Debounce counter to avoid sending vehicle commands before the vehicle is 
           at the desired height */
        if (takeoff_dbc_cnt > 0)
        {
            takeoff_dbc_cnt--;
        }
        else
        {
            takeoff_dbc_cnt = 0;
            takeoff_dbc = true;
        }

        if (takeoff_dbc && g_mav_veh_rngfdr_current_distance > 550 || g_mav_veh_rel_alt > 5500)
        {
            start_follow_mode = true;
        }
        
        if (start_follow_mode)
        {
            follow_mode();
        }
    }
}

/********************************************************************************
 * Function: VehicleController
 * Description: Constructor of the VehicleController clasds.
 ********************************************************************************/
VehicleController::VehicleController(void) {}

/********************************************************************************
 * Function: VehicleController
 * Description: Constructor of the VehicleController class.
 ********************************************************************************/
VehicleController::~VehicleController(void) {}

/********************************************************************************
 * Function: init
 * Description: Initial setup of vehicle controller.
 ********************************************************************************/
bool VehicleController::init(void)
{
    takeoff_dbc = false;
    start_follow_mode = false;
    takeoff_dbc_cnt = 500;

    return true;
}

/********************************************************************************
 * Function: loop
 * Description: Vehicle controller main loop.  Handles all
 ********************************************************************************/
void VehicleController::loop(void)
{
    dtrmn_veh_control_action();
}

/********************************************************************************
 * Function: shutdown
 * Description: Code needed to shutdown the vehicle controller.
 ********************************************************************************/
void VehicleController::shutdown(void)
{
    MavCmd::set_mode_land(SENDER_SYS_ID, SENDER_COMP_ID, TARGET_SYS_ID, TARGET_COMP_ID);
}
