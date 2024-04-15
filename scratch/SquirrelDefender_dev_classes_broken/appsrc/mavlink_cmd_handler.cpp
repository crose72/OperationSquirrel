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
 * Function: MavCmd
 * Description: Class constructor
 ********************************************************************************/
MavCmd::MavCmd(){}

/********************************************************************************
 * Function: ~MavCmd
 * Description: Class destructor
 ********************************************************************************/
MavCmd::~MavCmd(){}

/********************************************************************************
 * Function: takeoff_sequence
 * Description: Command vehicle to fly up to a specified relative altitude.
 ********************************************************************************/
void MavCmd::takeoff_sequence(float takeoff_alt)
{
    // Set flight mode to guided
    set_mode_GUIDED();

    // ARM the vehicle
    arm_vehicle();

    // Takeoff
    takeoff_GPS_long(takeoff_alt);
}

/********************************************************************************
 * Function: takeoff_LOCAL
 * Description: Takekoff to specified height.
 ********************************************************************************/
void MavCmd::takeoff_LOCAL(float pitch, float ascend_rate, float yaw, int32_t x, int32_t y, float z)
{
    // All relevant arguments for MAV_CMD_NAV_TAKEOFF: uint8_t frame, uint16_t command, float param1, float param2, float param3, float param4, int32_t x, int32_t y, float z
    send_cmd_int(MAV_FRAME_BODY_OFFSET_NED, MAV_CMD_NAV_TAKEOFF_LOCAL, pitch, 0, ascend_rate, yaw, x, y, z);
}

/********************************************************************************
 * Function: takeoff_GPS_long
 * Description: Takekoff to specified height using command_long.
 ********************************************************************************/
void MavCmd::takeoff_GPS_long(float alt)
{
    send_cmd_long(MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt);
}

/********************************************************************************
 * Function: arm_vehicle
 * Description: Arm vehicle.
 ********************************************************************************/
void MavCmd::arm_vehicle(void)
{
    // Arm throttle - param2 = 0 requires safety checks, param2 = 21196 allows
    // arming to override preflight checks and disarming in flight
    send_cmd_long(MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0);
}

/********************************************************************************
 * Function: disarm_vehicle
 * Description: Disarm vehicle.
 ********************************************************************************/
void MavCmd::disarm_vehicle(void)
{
    // Disarm throttle - param2 = 0 requires safety checks, param2 = 21196 allows
    // arming to override preflight checks and disarming in flight
    send_cmd_long(MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 1, 0, 0, 0, 0, 0);
}

/********************************************************************************
 * Function: set_mode_GUIDED
 * Description: Change mode to GUIDED to allow control from a companion computer.
 ********************************************************************************/
void MavCmd::set_mode_GUIDED(void)
{
    set_flight_mode(0, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, (float)FlightMode::GUIDED, 0);
}


/********************************************************************************
 * Function: set_mode_RTL
 * Description: Change mode to RTL and vehicle will return to launch location.
 ********************************************************************************/
void MavCmd::set_mode_RTL(void)
{
    set_flight_mode(0, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, (float)FlightMode::RTL, 0);
}

/********************************************************************************
 * Function: go_to_waypoint
 * Description: Commmand vehicle to move to a specified GPS location with 
 *              specific latitude, longtitude, and altitude.
 ********************************************************************************/
void MavCmd::go_to_waypoint(int32_t lat, int32_t lon, float alt)
{
    send_cmd_set_position_target_global_int(MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, (uint16_t)0b111111111000, lat, lon, alt, 0, 0, 0, 0, 0, 0, 0, 0); //mavlink_msg_command_int_pack(SENDER_SYS_ID, SENDER_COMP_ID, &msg, TARGET_SYS_ID, TARGET_COMP_ID, MAV_CMD_DO_SET_MODE, 0, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6, 0, 0, -35.3671 * 1e7, 149.1649 * 1e7, 0); is used for AUTO mode?
}

/********************************************************************************
 * Function: set_flight_mode
 * Description: Set the vehicle flight mode 
 ********************************************************************************/
 void MavCmd::set_flight_mode(uint8_t confirmation, float mode, float custom_mode, float custom_submode)
 {
    send_cmd_long(MAV_CMD_DO_SET_MODE, confirmation, mode, custom_mode, custom_submode, 0, 0, 0, 0);
 }

/********************************************************************************
 * Function: set_mav_msg_rate
 * Description: This function is specifically designed to send a 
 *              mavlink command long to request that a mavlink message ID is 
 *              sent at a desired rate.
 ********************************************************************************/
void MavCmd::set_mav_msg_rate(uint16_t mavlink_command, uint16_t msg_id, float msg_interval) 
{
    uint16_t len = 0;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t msg;

    mavlink_msg_command_long_pack(SENDER_SYS_ID, SENDER_COMP_ID, &msg, TARGET_SYS_ID, TARGET_COMP_ID, mavlink_command, 0, msg_id, msg_interval, 0, 0, 0, 0, 0);
    serial.offset_buffer(buffer, len, msg);
    serial.write_serial_port(buffer, len);
}

/********************************************************************************
 * Function: req_mav_msg
 * Description: This overloaded function is specifically designed to send a 
 *              mavlink command long torequest that a specific mavlink message 
 *              ID is sent.
 ********************************************************************************/
void MavCmd::req_mav_msg(uint16_t mavlink_command, uint16_t msg_id) 
{
    uint16_t len = 0;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t msg;

    mavlink_msg_command_long_pack(SENDER_SYS_ID, SENDER_COMP_ID, &msg, TARGET_SYS_ID, TARGET_COMP_ID, mavlink_command, 0, msg_id, 0, 0, 0, 0, 0, 0);
    serial.offset_buffer(buffer, len, msg);
    serial.write_serial_port(buffer, len);
}

/********************************************************************************
 * Function: send_cmd_int
 * Description: This overloaded function allows for various mavlink command 
 *              longs to be sent, including a mode change, takeoff, ARM throttle, 
 *              and more.
 ********************************************************************************/
void MavCmd::send_cmd_int(uint8_t frame, uint16_t command, float param1, float param2, float param3, float param4, int32_t x, int32_t y, float z) 
{
    uint16_t len = 0;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t msg;
    uint8_t current = 0;        // no used according to documentation, set 0
    uint8_t autocontinue = 0;   // no used according to documentation, set 0

    mavlink_msg_command_int_pack(SENDER_SYS_ID, SENDER_COMP_ID, &msg, TARGET_SYS_ID, TARGET_COMP_ID, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z);
    serial.offset_buffer(buffer, len, msg);
    serial.write_serial_port(buffer, len);
}

/********************************************************************************
 * Function: send_cmd_long
 * Description: This function sends mavlink command 
 *              long which can do a mode change, takeoff, ARM throttle, 
 *              and more.
 ********************************************************************************/
void MavCmd::send_cmd_long(uint16_t mavlink_command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7) 
{
    uint16_t len = 0;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t msg;

    mavlink_msg_command_long_pack(SENDER_SYS_ID, SENDER_COMP_ID, &msg, TARGET_SYS_ID, TARGET_COMP_ID, mavlink_command, confirmation, param1, param2, param3, param4, param5, param6, param7);
    serial.offset_buffer(buffer, len, msg);
    serial.write_serial_port(buffer, len);
}

/********************************************************************************
 * Function: send_cmd_set_position_target_global_int
 * Description: This function uses the mavlink message set position target 
 *              global int message to send a vehicle to a specific GPS location 
 *              using an external controller.
 ********************************************************************************/
void MavCmd::send_cmd_set_position_target_global_int(uint8_t coordinate_frame, uint16_t type_mask, int32_t lat_int, int32_t lon_int, float alt, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate) 
{
    uint16_t len = 0;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t msg;

    // There should be an _encode function we want to use instead of the pack function
    mavlink_msg_set_position_target_global_int_pack(SENDER_SYS_ID, SENDER_COMP_ID, &msg, 0, TARGET_SYS_ID, TARGET_COMP_ID, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, (uint16_t)0b111111111000, lat_int, lon_int, alt, 0, 0, 0, 0, 0, 0, 0, 0);
    serial.offset_buffer(buffer, len, msg);
    serial.write_serial_port(buffer, len);
}

/********************************************************************************
 * Function: send_cmd_set_attitude_target
 * Description: This function uses the mavlink message set attitude target to 
 *              allow for manually controlling the roll, pitch, yaw, and thrust 
 *              of a vehicle using an external controller.
 ********************************************************************************/
void MavCmd::send_cmd_set_attitude_target(mavlink_set_attitude_target_t *desired_attitude_target)
{
    uint16_t len = 0;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t msg;

    mavlink_msg_set_attitude_target_encode(SENDER_SYS_ID, SENDER_COMP_ID, &msg, desired_attitude_target);
    serial.offset_buffer(buffer, len, msg);
    serial.write_serial_port(buffer, len);
}

/********************************************************************************
 * Function: send_cmd_set_position_target_local_ned
 * Description: This function uses the mavlink message set position target to 
 *              allow for manually controlling the position, velocity, or
 *              acceleration of a vehicle using an external controller.
 ********************************************************************************/
void MavCmd::send_cmd_set_position_target_local_ned(mavlink_set_position_target_local_ned_t *desired_target)
{
    uint16_t len = 0;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t msg;

    mavlink_msg_set_position_target_local_ned_encode(SENDER_SYS_ID, SENDER_COMP_ID, &msg, desired_target);
    serial.offset_buffer(buffer, len, msg);
    serial.write_serial_port(buffer, len);
}