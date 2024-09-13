#pragma once

/********************************************************************************
 * @file    mavlink_msg_handler.h
 * @author  Cameron Rose
 * @date    6/7/2023
 ********************************************************************************/
#ifndef MAVLINK_PRINT_INFO_H
#define MAVLINK_PRINT_INFO_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include <mavlink.h>

/********************************************************************************
 * Imported objects
 ********************************************************************************/

/********************************************************************************
 * Exported objects
 ********************************************************************************/

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
void print_command_ack(mavlink_command_ack_t &command_ack, const char *term);
void print_distance_sensor(mavlink_distance_sensor_t &distance_sensor, const char *term);
void print_heartbeat(mavlink_heartbeat_t &heartbeat, const char *term);
void print_sys_status(mavlink_sys_status_t &sys_status, const char *term);
void print_gps_raw_int(mavlink_gps_raw_int_t &gps_raw_int, const char *term);
void print_attitude(mavlink_attitude_t &attitude, const char *term);
void print_global_position_int(mavlink_global_position_int_t &global_pos_int, const char *term);
void print_scaled_imu(mavlink_scaled_imu_t &scaled_imu, const char *term);
void print_raw_imu(mavlink_raw_imu_t &raw_imu, const char *term);
void print_param_request_read(mavlink_param_request_read_t &param_request_read, const char *term);
void print_request_data_stream(mavlink_request_data_stream_t &request_data_stream, const char *term);
void print_gps_global_origin(mavlink_gps_global_origin_t &gps_global_origin, const char *term);
void print_home_position(mavlink_home_position_t &home_position, const char *term);
void print_statustext(mavlink_statustext_t &statustext, const char *term);
void print_param_value(mavlink_param_value_t &param_value, const char *term);
void print_attitude(mavlink_attitude_t &attitude, const char *term);
void print_rc_channels_raw(mavlink_rc_channels_raw_t &rc_channels_raw, const char *term);
void print_local_position(mavlink_local_position_ned_t &local_position, const char *term);
void print_rc_channels_scaled(mavlink_rc_channels_scaled_t &rc_channels_scaled, const char *term);
void print_servo_output_raw(mavlink_servo_output_raw_t &servo_output_raw, const char *term);
void print_autopilot_version(mavlink_autopilot_version_t &autopilot_version, const char *term);
void print_attitude_quaternion(mavlink_attitude_quaternion_t &attitude_quaternion, const char *term);
void print_attitude_target(mavlink_attitude_target_t &attitude_target, const char *term);
void print_set_attitude_target(mavlink_set_attitude_target_t &set_attitude_target, const char *term);
void print_system_time(mavlink_system_time_t &system_time, const char *term);
void print_set_position_target_local_ned(mavlink_set_position_target_local_ned_t &set_position_target_local_ned, const char *term);
void print_position_target_local_ned(mavlink_position_target_local_ned_t &position_target_local_ned, const char *term);
void print_optical_flow(mavlink_optical_flow_t &optical_flow, const char *term);
void print_position_local_ned(const mavlink_local_position_ned_t &local_position, const char *term);

#endif // MAVLINK_PRINT_INFO_H