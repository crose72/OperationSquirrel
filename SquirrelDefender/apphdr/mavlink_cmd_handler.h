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
    static void takeoff_sequence(float takeoff_alt);
    static void arm_vehicle(void);
    static void disarm_vehicle(void);
    static void takeoff_LOCAL(float pitch, float ascend_rate, float yaw,int32_t x, int32_t y, float z);
    static void takeoff_GPS_long(float mav_veh_alt);
    static void takeoff_GPS(float alt);
    static void set_mode_GUIDED(void);
    static void set_mode_RTL(void);
    static void go_to_waypoint(int32_t lat, int32_t lon, float alt);
    static void set_flight_mode(uint8_t confirmation, float mode, float custom_mode, float custom_submode);
    static void set_mav_msg_rate(uint16_t msg_id, float msg_interval);                                           // For setting a message rate - command, msg id, message rate
    static void req_mav_msg(uint16_t msg_id);                                                                    // For requesting a mavlink message - command, msg id
    static void send_cmd_long(uint16_t mavlink_command, uint8_t confirmation, 
                    float param1, float param2, float param3, float param4, float param5, float param6, float param7);              // command, confirmation, param1 - param7
    static void send_cmd_int(uint8_t frame, uint16_t command, 
                    float param1, float param2, float param3, float param4, int32_t x, int32_t y, float z);
    static void send_cmd_set_position_target_global_int(uint8_t coordinate_frame, uint16_t type_mask, int32_t lat_int, int32_t lon_int, 
                    float alt, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate);           // coordinate_frame, type_mask, lat_int, lon_int, alt, vx, vy, vz, afx, afy, afz, yaw, yaw_rate
    static void send_cmd_set_attitude_target(mavlink_set_attitude_target_t *desired_attitude_target);                                      // type_mask, *q, body_roll_rate, body_pitch_rate, body_yaw_rate, thrust, *thrust_body
    static void send_cmd_set_position_target_local_ned(mavlink_set_position_target_local_ned_t *desired_target);  

private:
    static void send_mav_cmd(mavlink_message_t &msg){SerialComm::write_uart(msg);};
};


#endif // MAVLINK_COMMAND_HANDLER_H