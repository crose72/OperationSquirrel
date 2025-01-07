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
#include "common_inc.h"
#include "serial_comm.h"
#include <mavlink.h>
#include <common.h>

/********************************************************************************
 * Imported objects
 ********************************************************************************/
extern const uint8_t SENDER_SYS_ID;
extern const uint8_t SENDER_COMP_ID;
extern const uint8_t TARGET_SYS_ID;
extern const uint8_t TARGET_COMP_ID;
extern const int32_t MESSAGE_RATE_DEFAULT;
extern const int32_t MESSAGE_RATE_1Hz;

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
    static void takeoff_LOCAL(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, int32_t x, int32_t y, float z);
    static void takeoff_GPS_long(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, float alt);
    static void set_mode_LAND(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id);
    static void set_mode_GUIDED(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id);
    static void set_mode_GUIDED_NOGPS(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id);
    static void set_mode_RTL(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id);
    static void go_to_waypoint(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, int32_t lat, int32_t lon, float alt);
    static void set_flight_mode(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, 
                             uint8_t confirmation, float mode, float custom_mode, float custom_submode);
    static void set_mav_msg_rate(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, uint16_t msg_id, float msg_interval); // For setting a message rate - command, msg id, message rate
    static void req_mav_msg(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, uint16_t msg_id);                          // For requesting a mavlink message - command, msg id
    static void send_cmd_long(uint8_t sender_sys_id, uint8_t sender_comp_id, const mavlink_command_long_t& command_long); // command, confirmation, param1 - param7
    static void send_cmd_int(uint8_t sender_sys_id, uint8_t sender_comp_id, const mavlink_command_int_t& command_int);
    static void send_cmd_set_position_target_global_int(uint8_t sender_sys_id, uint8_t sender_comp_id, const mavlink_set_position_target_global_int_t* set_position_target_global_int); // coordinate_frame, type_mask, lat_int, lon_int, alt, vx, vy, vz, afx, afy, afz, yaw, yaw_rate
    static void send_cmd_set_attitude_target(mavlink_set_attitude_target_t *desired_attitude_target);                                                         // type_mask, *q, body_roll_rate, body_pitch_rate, body_yaw_rate, thrust, *thrust_body
    static void send_cmd_set_position_target_local_ned(mavlink_set_position_target_local_ned_t *desired_target);

private:
    static void send_mav_cmd(mavlink_message_t &msg) { SerialComm::write_uart(msg); };
};

#endif // MAVLINK_COMMAND_HANDLER_H
