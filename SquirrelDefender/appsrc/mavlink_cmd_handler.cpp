/********************************************************************************
 * @file    mavlink_cmd_handler.cpp
 * @author  Cameron Rose
 * @date    12/27/2023
 * @brief   Abstracts and simplifies the mavlink commands for use throughout the
 *          program.
 * 
            // Examples of commands than can be sent
            // MAV_CMD_DO_MOTOR_TEST instance, MOTOR_TEST_THROTTLE_TYPE, throttle, timeout, motor count, test order, empty
            // MAV_CMD_DO_FENCE_ENABLE enable, empty, etc
            // MAV_CMD_DO_SET_ROI, ROI mode, WP index, ROI index, empty, MAV_ROI__WPNEXT pitch, MAV_ROI__WPNEXT roll, MAV_ROI__WPNEXT yaw
            // MAV_CMD_DO_SET_PARAMETER param number, value
            // MAV_CMD_DO_SET_SERVO instance (servo #),PWM value
            // MAV_CMD_DO_SET_HOME 1 use current 0 use specified, empty, empty, yaw (NAN for default), lat, lon, altitude
            // MAV_CMD_DO_CHANGE_SPEED speed type, speed, throttle. reserved, etc (all set 0)
            // MAV_CMD_NAV_GUIDED_ENABLE enable (> 05f on), empty, etc
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "mavlink_cmd_handler.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: msg_handler
 * Description: Forgot the purpose of this, may be unnecessary.
 ********************************************************************************/
void msg_handler(uint16_t mavlink_command, uint16_t msg_id, float msg_interval) 
{
    uint16_t len = 0; // length of buffer
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN]; // define length of buffer
    mavlink_message_t msg; // initialize/*< [degE7] Longitude, expressed*/ the Madvlink message buffer

    mavlink_msg_command_long_pack(SENDER_SYS_ID, SENDER_COMP_ID, &msg, TARGET_SYS_ID, TARGET_COMP_ID, mavlink_command, 0, msg_id, msg_interval, 0, 0, 0, 0, 0);
    offset_buffer(buffer, len, msg);
    write_serial_port(buffer, len);
}

/********************************************************************************
 * Function: takeoff_sequence
 * Description: Command vehicle to fly up to a specified relative altitude.
 ********************************************************************************/
void takeoff_sequence(float takeoff_alt)
{
    // Set flight mode to guided
    // enable or disable custom mode (including guided), mode # (found in mode.h), custom submode, empty, etc
    send_cmd_long(MAV_CMD_DO_SET_MODE, 0, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 0, 0, 0, 0, 0); // guided = 4

    // Hand control over to the companion computer
    // send_cmd_long(MAV_CMD_NAV_GUIDED_ENABLE, 0, 1, 0, 0, 0, 0, 0, 0);
    
    // ARM the vehicle
    send_cmd_long(MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 1, 0, 0, 0, 0, 0);

    // Takeoff
    send_cmd_long(MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, takeoff_alt);
}

/********************************************************************************
 * Function: return_to_launch
 * Description: Change mode to RTL and vehicle will return to launch location.
 ********************************************************************************/
void return_to_launch(void)
{
    // Set flight mode to RTL
    send_cmd_long(MAV_CMD_DO_SET_MODE, 0, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6, 0, 0, 0, 0, 0);
}

/********************************************************************************
 * Function: go_to_waypoint
 * Description: Commmand vehicle to move to a specified GPS location with 
 *              specific latitude, longtitude, and altitude.
 ********************************************************************************/
void go_to_waypoint(int32_t lat, int32_t lon, float alt)
{
    send_cmd_set_position_target_global_int(MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, (uint16_t)0b111111111000, lat, lon, alt, 0, 0, 0, 0, 0, 0, 0, 0); //mavlink_msg_command_int_pack(SENDER_SYS_ID, SENDER_COMP_ID, &msg, TARGET_SYS_ID, TARGET_COMP_ID, MAV_CMD_DO_SET_MODE, 0, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6, 0, 0, -35.3671 * 1e7, 149.1649 * 1e7, 0); is used for AUTO mode?
}

/********************************************************************************
 * Function: send_cmd_long
 * Description: This overloaded function is specifically designed to send a 
 *              mavlink command long to request that a mavlink message ID is 
 *              sent at a desired rate.
 ********************************************************************************/
void send_cmd_long(uint16_t mavlink_command, uint16_t msg_id, float msg_interval) 
{
    uint16_t len = 0; // length of buffer
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN]; // define length of buffer
    mavlink_message_t msg; // initialize the Madvlink message buffer

    mavlink_msg_command_long_pack(SENDER_SYS_ID, SENDER_COMP_ID, &msg, TARGET_SYS_ID, TARGET_COMP_ID, mavlink_command, 0, msg_id, msg_interval, 0, 0, 0, 0, 0);
    offset_buffer(buffer, len, msg);
    write_serial_port(buffer, len);
}

/********************************************************************************
 * Function: send_cmd_long
 * Description: This overloaded function is specifically designed to send a 
 *              mavlink command long torequest that a specific mavlink message 
 *              ID is sent.
 ********************************************************************************/
void send_cmd_long(uint16_t mavlink_command, uint16_t msg_id) 
{
    uint16_t len = 0; // length of buffer
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN]; // define length of buffer
    mavlink_message_t msg; // initialize the Madvlink message buffer

    mavlink_msg_command_long_pack(SENDER_SYS_ID, SENDER_COMP_ID, &msg, TARGET_SYS_ID, TARGET_COMP_ID, mavlink_command, 0, msg_id, 0, 0, 0, 0, 0, 0);
    offset_buffer(buffer, len, msg);
    write_serial_port(buffer, len);
}

/********************************************************************************
 * Function: send_cmd_long
 * Description: This overloaded function allows for various mavlink command 
 *              longs to be sent, including a mode change, takeoff, ARM throttle, 
 *              and more.
 ********************************************************************************/
void send_cmd_long(uint16_t mavlink_command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7) 
{
    uint16_t len = 0; // length of buffer
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN]; // define length of buffer
    mavlink_message_t msg; // initialize the Madvlink message buffer

    mavlink_msg_command_long_pack(SENDER_SYS_ID, SENDER_COMP_ID, &msg, TARGET_SYS_ID, TARGET_COMP_ID, mavlink_command, confirmation, param1, param2, param3, param4, param5, param6, param7);
    offset_buffer(buffer, len, msg);
    write_serial_port(buffer, len);
}

/********************************************************************************
 * Function: send_cmd_set_position_target_global_int
 * Description: This function uses the mavlink message set position target 
 *              global int message to send a vehicle to a specific GPS location 
 *              using an external controller.
 ********************************************************************************/
void send_cmd_set_position_target_global_int(uint8_t coordinate_frame, uint16_t type_mask, int32_t lat_int, int32_t lon_int, float alt, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate) 
{
    uint16_t len = 0; // length of buffer
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN]; // define length of buffer
    mavlink_message_t msg; // initialize the Madvlink message buffer

    // There should be an _encode function we want to use instead of the pack function
    mavlink_msg_set_position_target_global_int_pack(SENDER_SYS_ID, SENDER_COMP_ID, &msg, 0, TARGET_SYS_ID, TARGET_COMP_ID, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, (uint16_t)0b111111111000, lat_int, lon_int, alt, 0, 0, 0, 0, 0, 0, 0, 0);
    offset_buffer(buffer, len, msg);
    write_serial_port(buffer, len);
}

/********************************************************************************
 * Function: send_cmd_set_attitude_target
 * Description: This function uses the mavlink message set attitude target to 
 *              allow for manually controlling the roll, pitch, yaw, and thrust 
 *              of a vehicle using an external controller.
 ********************************************************************************/
void send_cmd_set_attitude_target(mavlink_set_attitude_target_t *desired_attitude_target)
{
    uint16_t len = 0; // length of buffer
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN]; // define length of buffer
    mavlink_message_t msg; // initialize the Madvlink message buffer

    mavlink_msg_set_attitude_target_encode(SENDER_SYS_ID, SENDER_COMP_ID, &msg, desired_attitude_target);
    offset_buffer(buffer, len, msg);
    write_serial_port(buffer, len);
}

/********************************************************************************
 * Function: send_cmd_set_position_target_local_ned
 * Description: This function uses the mavlink message set position target to 
 *              allow for manually controlling the position, velocity, and
 *              acceleration of a vehicle using an external controller.
 ********************************************************************************/
void send_cmd_set_position_target_local_ned(mavlink_set_position_target_local_ned_t *desired_position_target)
{
    uint16_t len = 0; // length of buffer
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN]; // define length of buffer
    mavlink_message_t msg; // initialize the Madvlink message buffer

    mavlink_msg_set_position_target_local_ned_encode(SENDER_SYS_ID, SENDER_COMP_ID, &msg, desired_position_target);
    offset_buffer(buffer, len, msg);
    write_serial_port(buffer, len);
}

