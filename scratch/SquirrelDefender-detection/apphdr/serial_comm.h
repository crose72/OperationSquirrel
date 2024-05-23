/********************************************************************************
 * @file    serial_comm.h
 * @author  Cameron Rose
 * @date    12/27/2023
 ********************************************************************************/
#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include <fcntl.h> // for open, fcntl, etc.
#include <termios.h> // for terminal I/O
#include <unistd.h> // for POSIX API functions
#include <sys/ioctl.h> // for ioctl
#include <sys/socket.h>

// For TCP only
#include <sys/socket.h> // for socket operations
#include <arpa/inet.h> // for inet_addr
#include <netinet/in.h> // for sockaddr_in

/********************************************************************************
 * Imported objects
 ********************************************************************************/

/********************************************************************************
 * Exported objects
 ********************************************************************************/

/********************************************************************************
 * Function prototypes
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
