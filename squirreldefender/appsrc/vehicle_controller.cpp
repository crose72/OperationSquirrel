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
#include "common_inc.h"
#include "vehicle_controller.h"
#include "target_tracking.h"
#include "mav_data_hub.h"
#include "mav_utils.h"
#include "velocity_controller.h"
#include "system_controller.h"
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
bool takeoff_dbc;
bool start_follow_mode;
uint16_t takeoff_dbc_cnt;
float desired_veh_alt_m;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/
uint16_t min_mission_alt_cm = (uint16_t)550;
int32_t min_mission_alt_mm = (int32_t)5500;

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

    target_velocity[0] = g_ctrl_vel_x_cmd;
    target_velocity[1] = g_ctrl_vel_y_cmd;
    target_velocity[2] = g_ctrl_vel_z_cmd;

    if (g_tgt_valid)
    {
        MavMotion::cmd_velocity_NED(SENDER_SYS_ID, SENDER_COMP_ID, TARGET_SYS_ID, TARGET_COMP_ID, target_velocity, g_ctrl_yaw_cmd);
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
        MavCmd::takeoff_gps(SENDER_SYS_ID, SENDER_COMP_ID, TARGET_SYS_ID, TARGET_COMP_ID, desired_veh_alt_m);
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

        if (takeoff_dbc && (g_mav_rngfndr_dist_cm > min_mission_alt_cm || g_mav_gps_alt_rel > min_mission_alt_mm))
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
 * Description: Constructor
 ********************************************************************************/
VehicleController::VehicleController(void) {}

/********************************************************************************
 * Function: VehicleController
 * Description: Destructor
 ********************************************************************************/
VehicleController::~VehicleController(void) {}

/********************************************************************************
 * Function: init
 * Description: Initial setup of vehicle controller.
 ********************************************************************************/
bool VehicleController::init(void)
{
    ParamReader mission_params("../params.json");

    takeoff_dbc_cnt = mission_params.get_float_param("mission_params.mission_start_delay_count");
    min_mission_alt_cm = mission_params.get_float_param("mission_params.min_mission_alt_cm");
    min_mission_alt_mm = mission_params.get_float_param("mission_params.min_mission_alt_mm");
    desired_veh_alt_m = mission_params.get_float_param("mission_params.takeoff_alt_m");

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
    // When operating with OSRemote we don't want it to land every time
    // MavCmd::set_mode_land(SENDER_SYS_ID, SENDER_COMP_ID, TARGET_SYS_ID, TARGET_COMP_ID);
}
