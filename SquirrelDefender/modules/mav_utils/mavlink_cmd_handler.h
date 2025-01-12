#pragma once

/********************************************************************************
 * @file    mavlink_cmd_handler.h
 * @author  Cameron Rose
 * @date    6/7/2023
 * @brief   Mavlink command handler header
 ********************************************************************************/
#ifndef MAVLINK_COMMAND_HANDLER_H
#define MAVLINK_COMMAND_HANDLER_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "uart_utils.h"
#include <mavlink.h>
#include <common.h>

/********************************************************************************
 * Imported objects
 ********************************************************************************/

/********************************************************************************
 * Exported objects
 ********************************************************************************/

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
class MavCmd
{
public:
    MavCmd();
    ~MavCmd();

    // Public member functions
    static void arm_vehicle(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id);
    static void disarm_vehicle(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id);
    static void takeoff_local(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, int32_t x, int32_t y, float z);
    static void takeoff_gps(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, float alt);
    static void set_mode_land(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id);
    static void set_mode_guided(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id);
    static void set_mode_guided_nogps(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id);
    static void set_mode_rtl(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id);
    static void go_to_waypoint(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, int32_t lat, int32_t lon, float alt);
    static void set_flight_mode(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, 
                             uint8_t confirmation, float mode, float custom_mode, float custom_submode);
    static void set_mav_msg_rate(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, uint16_t msg_id, float msg_interval); // For setting a message rate - command, msg id, message rate
    static void req_mav_msg(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, uint16_t msg_id);                          // For requesting a mavlink message - command, msg id
    static void send_cmd_position_target(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_system, uint8_t target_component, 
                                      float x, float y, float z, float yaw, uint16_t type_mask, uint8_t coordinate_frame);
    static void send_cmd_velocity_target(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_system, uint8_t target_component, 
                                      float vx, float vy, float vz, float yaw, uint16_t type_mask, uint8_t coordinate_frame);                                  
    static void send_cmd_acccel_target(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_system, uint8_t target_component, 
                                      float ax, float ay, float az, float yaw, float yaw_rate, uint16_t type_mask, uint8_t coordinate_frame);
    static void send_cmd_long(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, 
                              uint16_t mavlink_command, uint8_t confirmation,
                              float cmd_long_param1, float cmd_long_param2, float cmd_long_param3, float cmd_long_param4, 
                              float cmd_long_param5, float cmd_long_param6, float cmd_long_param7); // command, confirmation, param1 - param7
    static void send_cmd_int(uint8_t sender_sys_id, uint8_t sender_comp_id, const mavlink_command_int_t& command_int);
    static void send_cmd_set_position_target_global_int(uint8_t sender_sys_id, uint8_t sender_comp_id, const mavlink_set_position_target_global_int_t* set_position_target_global_int); // coordinate_frame, type_mask, lat_int, lon_int, alt, vx, vy, vz, afx, afy, afz, yaw, yaw_rate
    static void send_cmd_set_position_target_local_ned(uint8_t sender_sys_id, uint8_t sender_comp_id, 
                                                    uint8_t target_system, uint8_t target_component,
                                                    float x, float y, float z, float vx, float vy, float vz, 
                                                    float afx, float afy, float afz, float yaw, float yaw_rate,
                                                    uint16_t type_mask, uint8_t coordinate_frame);
    static void send_cmd_set_position_target_local_ned(mavlink_set_position_target_local_ned_t *desired_target);
    static void send_cmd_set_attitude_target(mavlink_set_attitude_target_t *desired_attitude_target); // type_mask, *q, body_roll_rate, body_pitch_rate, body_yaw_rate, thrust, *thrust_body

private:
    static void send_mav_cmd(mavlink_message_t &msg) { SerialComm::write_uart(msg); };
    static bool start_mav_comm(void) { return SerialComm::start_uart_comm(); }; // Open up uart port for mavlink messages
    static void stop_mav_comm(void) { SerialComm::stop_uart_comm(); };          // Stop mavlink comms on uart port
    static uint8_t read_mav_msg(void) { return SerialComm::read_uart(); };      // Read a byte
    static void subscribe(uint16_t msg_id, float msg_interval);                 // Subscribe to a mavlink message at desired rate
};

#endif // MAVLINK_COMMAND_HANDLER_H
