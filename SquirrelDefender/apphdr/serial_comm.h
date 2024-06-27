#pragma once

/********************************************************************************
 * @file    serial_comm.h
 * @author  Cameron Rose
 * @date    6/7/2023
 ********************************************************************************/
#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <cstdio>  // for perror
#include <cstdint> // for uint8_t, uint16_t, etc.
#include <termios.h>
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
 class SerialComm
 {
    public:
        SerialComm();
        ~SerialComm();

        static bool start_uart_comm(void);
        static void stop_uart_comm(void);
        static void write_uart(mavlink_message_t &msg);
        static uint8_t read_uart(void);
        static int bytes_available(void);

    private:
        static void offset_buffer(uint8_t* buffer, uint16_t &len, mavlink_message_t &msg);
        static void clear_buffer(uint8_t* buffer, uint16_t len);

 };
 /*
bool start_uart_comm(void);
void write_to_uart(uint8_t* buffer, uint16_t len);
void send_mav_msg(uint8_t* buffer, uint16_t len);
uint8_t read_mav_msg(void);
void SerialComm::offset_buffer(uint8_t* buffer, uint16_t &len, mavlink_message_t &msg);
void clear_buffer(uint8_t* buffer, uint16_t len);
void stop_serial_comm(void);*/

#endif // SERIAL_COMM_H
