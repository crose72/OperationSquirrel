#pragma once

/********************************************************************************
 * @file    mav_serial.h
 * @author  Cameron Rose
 * @date    1/22/2025
 ********************************************************************************/
#ifndef MAV_SERIAL_H
#define MAV_SERIAL_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include <mavlink.h>
#include <common.h>

/********************************************************************************
 * Typedefs / Enums / Structs
 ********************************************************************************/

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
class MavSerial
{
public:
    MavSerial();
    ~MavSerial();

    static bool start_mav_comm(void);
    static void stop_mav_comm(void);
    static void write_mav_msg(mavlink_message_t &msg);
    static uint8_t read_mav_msg(void);
    static int bytes_available(void);

private:
    static void offset_buffer(uint8_t *buffer, uint16_t &len, mavlink_message_t &msg);
    static void clear_buffer(uint8_t *buffer, uint16_t len);
};

#endif // MAV_SERIAL_H
