/********************************************************************************
 * @file    serial_port_handler.cpp
 * @author  Cameron Rose
 * @date    12/27/2023
 * @brief   Configure and enable serial ports for simulated and real serial
 *          data.  TCP is used for SITL and UART is used for the hardware drone.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "serial_port_handler.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/
#ifdef USE_UART
    #define SERIAL_PORT "/dev/ttyTHS1"
    #define BAUD_RATE B115200
    int serial_port = 0;
#elif USE_TCP
    #define _POSIX_C_SOURCE 200809L // enable certain POSIX-specific functionality
    #define SIM_IP "127.0.0.1" // IP address of SITL simulation
    #define SIM_PORT 5762      // Port number used by SITL simulation
    struct sockaddr_in sim_addr;
    int sock = 0;
#else
    #error "Please define either USE_UART or USE_TCP to enable the corresponding functionality."
#endif
/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: SerialPort
 * Description: Class constructor
 ********************************************************************************/
SerialPort::SerialPort(void){}

/********************************************************************************
 * Function: ~SerialPort
 * Description: Class destructor
 ********************************************************************************/
SerialPort::~SerialPort(void){}

/********************************************************************************
 * Function: open_serial_port
 * Description: Configure and make available the serial port to be used for 
 *              UART comms.
 ********************************************************************************/
void SerialPort::open_serial_port(void)
{
    #ifdef USE_UART
        // Open the serial port for reading and writing
        serial_port = open(SERIAL_PORT, O_RDWR);
        if (serial_port < 0) 
        {
            fprintf(stderr,"Error opening serial port\n");
        }

        // Configure the serial port
        struct termios serial_config;
        tcgetattr(serial_port, &serial_config);
        serial_config.c_cflag = BAUD_RATE | CS8 | CLOCAL | CREAD;
        serial_config.c_iflag = IGNPAR;
        serial_config.c_oflag = 0;
        serial_config.c_lflag = 0;
        tcflush(serial_port, TCIFLUSH);
        tcsetattr(serial_port, TCSANOW, &serial_config);

    #elif USE_TCP
        // Create a socket for the TCP connection
        sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0)
        {
            fprintf(stderr,"Error creating socket\n");
        }

        // Configure the address for the TCP connection
        sim_addr.sin_family = AF_INET;
        sim_addr.sin_addr.s_addr = inet_addr(SIM_IP);
        sim_addr.sin_port = htons(SIM_PORT);

        // Connect to the SITL simulation
        if (connect(sock, (struct sockaddr *)&sim_addr, sizeof(sim_addr)) < 0)
        {
            fprintf(stderr,"Error connecting to sitl\n");
        }

        // Set the socket to non-blocking mode
        int flags = fcntl(sock, F_GETFL, 0);
        fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    #else

    #endif

}

/********************************************************************************
 * Function: write_serial_port
 * Description: Send a buffer of bytes on the serial port.
 ********************************************************************************/
void SerialPort::write_serial_port(uint8_t* buffer, uint16_t len)
{
    uint16_t n;

    #ifdef USE_UART
        n = write(serial_port, buffer, len);

        if (MAVLINK_MAX_PACKET_LEN < len)
        {
            fprintf(stderr,"Buffer too large\n");
        }

        if (n < 0) 
        {
            fprintf(stderr,"Error writing to serial port\n");
        }

    #elif USE_TCP
        n = sendto(sock, buffer, len, 0, (struct sockaddr*)&sim_addr, sizeof(sim_addr));

        if (MAVLINK_MAX_PACKET_LEN < len)
        {
            fprintf(stderr,"Buffer too large\n");
        }

        if (n < 0) 
        {
            fprintf(stderr,"Error writing to serial port\n");
        }

    #else
        
    #endif

    clear_buffer(buffer, len);
}

/********************************************************************************
 * Function: read_serial_port
 * Description: Read some bytes from the serial port.
 ********************************************************************************/
uint8_t SerialPort::read_serial_port(void) 
{
    uint8_t byte;

    #ifdef USE_UART
        
        if (read(serial_port, &byte, 1) == -1) 
        {
            fprintf(stderr,"Error reading from serial port\n");
        }

    #elif USE_TCP
        ssize_t n = recv(sock, &byte, sizeof(byte), 0);

        if (n < 0) 
        {
            fprintf(stderr, "Error reading from port\n");
        }

        
    #else
        
    #endif

    return byte;
}

/********************************************************************************
 * Function: close_serial_port
 * Description: Close the serial port so it is no longer able to transmit.
 ********************************************************************************/
void SerialPort::close_serial_port(void)
{
    #ifdef USE_UART
        close(serial_port);

    #elif USE_TCP
        close(sock);
    #else
        
    #endif

}

/********************************************************************************
 * Function: offset_buffer
 * Description: Offset the buffer by the its length to avoid overlapping buffers.
 ********************************************************************************/
void SerialPort::offset_buffer(uint8_t* buffer, uint16_t &len, mavlink_message_t &msg)
{
    len = len + mavlink_msg_to_send_buffer(&buffer[len], &msg);
}

/********************************************************************************
 * Function: clear_buffer
 * Description: Clear the buffer
 ********************************************************************************/
void SerialPort::clear_buffer(uint8_t* buffer, uint16_t len)
{
    memset(buffer, 0, len);
}
