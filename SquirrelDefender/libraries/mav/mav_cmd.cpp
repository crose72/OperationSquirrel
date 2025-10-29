/********************************************************************************
 * @file    mav_cmd.cpp
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Utilities for sending mavlink commands over UART serial pins.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "mav_cmd.h"

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: MavCmd
 * Description: Class constructor
 ********************************************************************************/
MavCmd::MavCmd() {};

/********************************************************************************
 * Function: ~MavCmd
 * Description: Class destructor
 ********************************************************************************/
MavCmd::~MavCmd() {};

/********************************************************************************
 * Function: takeoff_local
 * Description: Takekoff to specified height.
 ********************************************************************************/
void MavCmd::takeoff_local(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, int32_t x, int32_t y, float z)
{
    mavlink_command_int_t command_int;

    command_int.target_system = target_sys_id;
    command_int.target_component = target_sys_id;
    command_int.frame = MAV_FRAME_BODY_OFFSET_NED;
    command_int.command = MAV_CMD_NAV_takeoff_local;
    command_int.x = x;
    command_int.y = y;
    command_int.z = z;

    send_cmd_int(sender_sys_id, sender_comp_id, command_int);
}

/********************************************************************************
 * Function: takeoff_gps
 * Description: Takekoff to specified height using command_long->
 ********************************************************************************/
void MavCmd::takeoff_gps(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, float alt)
{
    send_cmd_long(sender_sys_id, sender_comp_id, target_sys_id, target_comp_id, MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt);
}

/********************************************************************************
 * Function: arm_vehicle
 * Description: Arm vehicle.
 ********************************************************************************/
void MavCmd::arm_vehicle(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id)
{
    // Arm throttle - param2 = 0 requires safety checks, param2 = 21196 allows
    // arming to override preflight checks and disarming in flight
    send_cmd_long(sender_sys_id, sender_comp_id, target_sys_id, target_comp_id, MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0);
}

/********************************************************************************
 * Function: disarm_vehicle
 * Description: Disarm vehicle.
 ********************************************************************************/
void MavCmd::disarm_vehicle(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id)
{
    // Disarm throttle - param2 = 0 requires safety checks, param2 = 21196 allows
    // arming to override preflight checks and disarming in flight
    send_cmd_long(sender_sys_id, sender_comp_id, target_sys_id, target_comp_id, MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 1, 0, 0, 0, 0, 0);
}

/********************************************************************************
 * Function: set_mode_land
 * Description: Change mode to LAND.
 ********************************************************************************/
void MavCmd::set_mode_land(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id)
{
    set_flight_mode(sender_sys_id, sender_comp_id, target_sys_id, target_comp_id,
                    0, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, (float)FlightMode::LAND, 0);
}

/********************************************************************************
 * Function: set_mode_guided
 * Description: Change mode to GUIDED to allow control from a companion computer.
 ********************************************************************************/
void MavCmd::set_mode_guided(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id)
{
    set_flight_mode(sender_sys_id, sender_comp_id, target_sys_id, target_comp_id,
                    0, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, (float)FlightMode::GUIDED, 0);
}

/********************************************************************************
 * Function: set_mode_guided_nogps
 * Description: Change mode to GUIDED_NOGPS to allow control from a companion computer.
 ********************************************************************************/
void MavCmd::set_mode_guided_nogps(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id)
{
    set_flight_mode(sender_sys_id, sender_comp_id, target_sys_id, target_comp_id,
                    0, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, (float)FlightMode::GUIDED_NOGPS, 0);
}

/********************************************************************************
 * Function: set_mode_rtl
 * Description: Change mode to RTL and vehicle will return to launch location.
 ********************************************************************************/
void MavCmd::set_mode_rtl(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id)
{
    set_flight_mode(sender_sys_id, sender_comp_id, target_sys_id, target_comp_id,
                    0, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, (float)FlightMode::RTL, 0);
}

/********************************************************************************
 * Function: set_flight_mode
 * Description: Set the vehicle flight mode
 ********************************************************************************/
void MavCmd::set_flight_mode(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id,
                             uint8_t confirmation, float mode, float custom_mode, float custom_submode)
{
    send_cmd_long(sender_sys_id, sender_comp_id, target_sys_id, target_comp_id, MAV_CMD_DO_SET_MODE, confirmation, mode, custom_mode, custom_submode, 0, 0, 0, 0);
}

/********************************************************************************
 * Function: set_mav_msg_rate
 * Description: This function is specifically designed to send a
 *              mavlink command long to request that a mavlink message ID ims
 *              sent at a desired rate.
 ********************************************************************************/
void MavCmd::set_mav_msg_rate(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, uint16_t msg_id, float msg_interval)
{
    send_cmd_long(sender_sys_id, sender_comp_id, target_sys_id, target_comp_id, MAV_CMD_SET_MESSAGE_INTERVAL, 0, msg_id, msg_interval, 0, 0, 0, 0, 1);
}

/********************************************************************************
 * Function: req_mav_msg
 * Description: This overloaded function is specifically designed to send a
 *              mavlink command long torequest that a specific mavlink message
 *              ID is sent.
 ********************************************************************************/
void MavCmd::req_mav_msg(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, uint16_t msg_id)
{
    send_cmd_long(sender_sys_id, sender_comp_id, target_sys_id, target_comp_id, MAV_CMD_REQUEST_MESSAGE, 0, msg_id, 0, 0, 0, 0, 0, 1);
}

/********************************************************************************
 * Function: send_cmd_int
 * Description: This overloaded function allows for various mavlink command
 *              longs to be sent, including a mode change, takeoff, ARM throttle,
 *              and more.
 ********************************************************************************/
void MavCmd::send_cmd_int(uint8_t sender_sys_id, uint8_t sender_comp_id, const mavlink_command_int_t &command_int)
{
    mavlink_message_t msg;
    uint8_t command_int_current = 0;      // no used according to documentation, set 0
    uint8_t command_int_autocontinue = 0; // no used according to documentation, set 0

    mavlink_msg_command_int_encode(sender_sys_id, sender_comp_id, &msg, &command_int);
    send_mav_cmd(msg);
}

/********************************************************************************
 * Function: send_cmd_long
 * Description: This function sends mavlink command
 *              long which can do a mode change, takeoff, ARM throttle,
 *              and more.
 ********************************************************************************/
void MavCmd::send_cmd_long(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id,
                           uint16_t mavlink_command, uint8_t confirmation,
                           float cmd_long_param1, float cmd_long_param2, float cmd_long_param3, float cmd_long_param4,
                           float cmd_long_param5, float cmd_long_param6, float cmd_long_param7)
{
    mavlink_message_t msg;
    mavlink_command_long_t command_long;

    command_long.target_system = target_sys_id;
    command_long.target_component = target_comp_id;
    command_long.command = mavlink_command;
    command_long.confirmation = confirmation;
    command_long.param1 = cmd_long_param1;
    command_long.param2 = cmd_long_param2;
    command_long.param3 = cmd_long_param3;
    command_long.param4 = cmd_long_param4;
    command_long.param5 = cmd_long_param5;
    command_long.param6 = cmd_long_param6;
    command_long.param7 = cmd_long_param7;

    mavlink_msg_command_long_encode(sender_sys_id, sender_comp_id, &msg, &command_long);
    send_mav_cmd(msg);
}

/********************************************************************************
 * Function: read_param
 * Description: This function sends mavlink message to read a flight controller
 *              internal parameter.
 ********************************************************************************/
void MavCmd::read_param(uint8_t sender_sys_id,
                        uint8_t sender_comp_id,
                        uint8_t target_sys_id,
                        uint8_t target_comp_id,
                        const char *param_id,
                        int16_t param_index)
{
    mavlink_message_t msg;
    mavlink_param_request_read_t req;

    req.target_system = target_sys_id;
    req.target_component = target_comp_id;

    // Choose between name or index
    if (param_id && param_id[0] != '\0')
    {
        // Use name
        strncpy(req.param_id, param_id, sizeof(req.param_id));
        req.param_index = -1; // tells FCU to use param_id
    }
    else
    {
        // Use index
        req.param_index = param_index;
        req.param_id[0] = '\0'; // name ignored
    }

    // Encode and send
    mavlink_msg_param_request_read_encode(sender_sys_id, sender_comp_id, &msg, &req);
    send_mav_cmd(msg);
}