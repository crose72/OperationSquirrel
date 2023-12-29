/********************************************************************************
 * @file    mavlink_cmd_handler.h
 * @author  Cameron Rose
 * @date    12/27/2023
 * @brief   Mavlink command handler header
 ********************************************************************************/
#ifndef MAVLINK_COMMAND_HANDLER_H
#define MAVLINK_COMMAND_HANDLER_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "serial_port_handler.h"

/********************************************************************************
 * Imported objects
 ********************************************************************************/
extern const uint8_t SENDER_SYS_ID;
extern const uint8_t SENDER_COMP_ID;
extern const uint8_t TARGET_SYS_ID;
extern const uint8_t TARGET_COMP_ID;
extern const int32_t MESSAGE_RATE_DEFAULT;
extern const int32_t MESSAGE_RATE_1000us;

/********************************************************************************
 * Exported objects
 ********************************************************************************/

/********************************************************************************
 * Function prototypes
 ********************************************************************************/
void takeoff_sequence(float takeoff_alt);
void return_to_launch(void);
void go_to_waypoint(int32_t lat, int32_t lon, float alt);
void send_cmd_long(uint16_t, uint16_t); // FOr requesting a mavlink message - command, msg id
void send_cmd_long(uint16_t, uint16_t, float); // For setting a message rate - command, msg id, message rate
void send_cmd_long(uint16_t, uint8_t, float, float, float, float, float, float, float); // command, confirmation, param1 - param7
void send_cmd_set_position_target_global_int(uint8_t, uint16_t, int32_t, int32_t, float, float, float, float, float, float, float, float, float); // coordinate_frame, type_mask, lat_int, lon_int, alt, vx, vy, vz, afx, afy, afz, yaw, yaw_rate
void send_cmd_set_attitude_target(mavlink_set_attitude_target_t *); // type_mask, *q, body_roll_rate, body_pitch_rate, body_yaw_rate, thrust, *thrust_body
void send_cmd_set_position_target_local_ned(mavlink_set_position_target_local_ned_t *); // coordinate_frame, type_mask, x, y, z, vx, vy, vz, afx, afy, afz, yaw, yaw_rate

#endif // MAVLINK_COMMAND_HANDLER_H