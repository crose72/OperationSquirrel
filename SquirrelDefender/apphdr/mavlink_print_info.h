/********************************************************************************
 * @file    mavlink_msg_handler.h
 * @author  Cameron Rose
 * @date    12/27/2023
 ********************************************************************************/
#ifndef MAVLINK_PRINT_INFO_H
#define MAVLINK_PRINT_INFO_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"

/********************************************************************************
 * Imported objects
 ********************************************************************************/

/********************************************************************************
 * Exported objects
 ********************************************************************************/

/********************************************************************************
 * Function prototypes
 ********************************************************************************/
void print_heartbeat(mavlink_heartbeat_t &heartbeat);
void print_sys_status(mavlink_sys_status_t &sys_status);
void print_gps_raw_int(mavlink_gps_raw_int_t &gps_raw_int);
void print_attitude(mavlink_attitude_t &attitude);
void print_global_position_int(mavlink_global_position_int_t &global_pos_int);
void print_scaled_imu(mavlink_scaled_imu_t &scaled_imu);
void print_raw_imu(mavlink_raw_imu_t &raw_imu);
void print_command_ack(mavlink_command_ack_t &command_ack);
void print_param_request_read(mavlink_param_request_read_t &param_request_read);
void print_request_data_stream(mavlink_request_data_stream_t &request_data_stream);
void print_gps_global_origin(mavlink_gps_global_origin_t &gps_global_origin);
void print_home_position(mavlink_home_position_t &home_position);
void print_statustext(mavlink_statustext_t &statustext);
void print_param_value(mavlink_param_value_t &param_value);
void print_attitude(mavlink_attitude_t &attitude);
void print_rc_channels_raw(mavlink_rc_channels_raw_t &rc_channels_raw);
void print_local_position(mavlink_local_position_ned_t &local_position);
void print_rc_channels_scaled(mavlink_rc_channels_scaled_t &rc_channels_scaled);
void print_servo_output_raw(mavlink_servo_output_raw_t& servo_output_raw);
void print_autopilot_version(mavlink_autopilot_version_t &autopilot_version);
void print_attitude_quaternion(mavlink_attitude_quaternion_t &attitude_quaternion);
void print_attitude_target(mavlink_attitude_target_t &attitude_target);
void print_system_time(mavlink_system_time_t &system_time);
void print_set_position_target_local_ned(mavlink_set_position_target_local_ned_t &set_position_target_local_ned);
void print_position_target_local_ned(mavlink_position_target_local_ned_t &position_target_local_ned);
//void print_protocol_capability(mavlink_protocol_capability_t& protocol_;capability);

#endif // MAVLINK_PRINT_INFO_H