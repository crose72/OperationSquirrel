#pragma once

/********************************************************************************
 * @file    mav_cmd.h
 * @author  Cameron Rose
 * @date    1/22/2025
 ********************************************************************************/
#ifndef MAV_CMD_H
#define MAV_CMD_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "mav_serial.h"
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
    static void set_flight_mode(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, 
                             uint8_t confirmation, float mode, float custom_mode, float custom_submode);
    static void set_mav_msg_rate(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, uint16_t msg_id, float msg_interval); // For setting a message rate - command, msg id, message rate
    static void req_mav_msg(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, uint16_t msg_id);                          // For requesting a mavlink message - command, msg id
    static void send_cmd_long(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, 
                              uint16_t mavlink_command, uint8_t confirmation,
                              float cmd_long_param1, float cmd_long_param2, float cmd_long_param3, float cmd_long_param4, 
                              float cmd_long_param5, float cmd_long_param6, float cmd_long_param7); // command, confirmation, param1 - param7
    static void send_cmd_int(uint8_t sender_sys_id, uint8_t sender_comp_id, const mavlink_command_int_t& command_int);
    
private:
    static void send_mav_cmd(mavlink_message_t &msg) { MavSerial::write_mav_msg(msg); };
    static bool start_mav_comm(void) { return MavSerial::start_mav_comm(); }; // Open up uart port for mavlink messages
    static void stop_mav_comm(void) { MavSerial::stop_mav_comm(); };          // Stop mavlink comms on uart port
    static uint8_t read_mav_msg(void) { return MavSerial::read_mav_msg(); };      // Read a byte
    static void subscribe(uint16_t msg_id, float msg_interval);                 // Subscribe to a mavlink message at desired rate
};

#endif // MAV_CMD_H
