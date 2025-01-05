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
#include <cstdint> // for uint8_t, uint16_t, etc.
#include <mavlink.h>
#include <common.h>

#ifdef BLD_JETSON_B01 || WSL // for linux

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <cstdio>  // for perror
#include <cstdint> // for uint8_t, uint16_t, etc.
#include <termios.h>

#elif BLD_WIN

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
    static void offset_buffer(uint8_t *buffer, uint16_t &len, mavlink_message_t &msg);
    static void clear_buffer(uint8_t *buffer, uint16_t len);
};

#endif // SERIAL_COMM_H
