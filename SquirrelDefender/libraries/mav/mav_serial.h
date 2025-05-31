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
#include <cstdint> // for uint8_t, uint16_t, etc.
#include <mavlink.h>
#include <common.h>

#ifdef BLD_LINUX_SERIAL // for linux

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <cstdio>  // for perror
#include <cstdint> // for uint8_t, uint16_t, etc.
#include <termios.h>

#elif defined(BLD_WIN)

#include <winsock2.h> // For TCP/IP sockets
#include <ws2tcpip.h>
#include <windows.h>               // For COM ports (serial communication)
#pragma comment(lib, "Ws2_32.lib") // Link WinSock2 library for TCP
#include <windows.h>               // For WinAPI functions and types (CreateFile, HANDLE, DCB, etc.)
#include <stdio.h>                 // For standard input/output (e.g., printf)
#include <stdlib.h>                // For standard functions
#include <string.h>                // For string manipulation

#endif

/********************************************************************************
 * Imported objects
 ********************************************************************************/

/********************************************************************************
 * Exported objects
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
