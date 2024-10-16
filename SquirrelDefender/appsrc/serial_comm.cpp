/********************************************************************************
 * @file    serial_port_handler.cpp
 * @author  Cameron Rose
 * @date    6/7/2023
 * @brief   Configure and enable serial ports for simulated and real serial
 *          data.  TCP is used for SITL and UART is used for the hardware drone.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "serial_comm.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/
#ifdef USE_UART

// Do nothing

#elif USE_TCP

struct sockaddr_in sim_addr;

#else

#error "Please define USE_UART or USE_TCP."

#endif

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/
#ifdef USE_UART

#define SERIAL_PORT "/dev/ttyTHS1"
#define BAUD_RATE B115200
// #define SERIAL_PORT "/dev/ttyUSB0" // for SiK radio
// #define BAUD_RATE B57600 // SiK radio baud rate

#elif USE_TCP

#define _POSIX_C_SOURCE 200809L // enable certain POSIX-specific functionality
#define SIM_IP "127.0.0.1"      // IP address of SITL simulation
#define SIM_PORT 5762           // Port number used by SITL simulation

#else

#error "Please define USE_UART or USE_TCP."

#endif

/********************************************************************************
 * Object definitions
 ********************************************************************************/
#ifdef USE_UART
int uart_fd; // UART file descriptor
int serial_port = 0;
#elif USE_TCP
int tcp_socket_fd; // TCP socket file descriptor
int sock;
#else
#error "Please define USE_UART or USE_TCP."
#endif

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: SerialComm
 * Description: Class constructor
 ********************************************************************************/
SerialComm::SerialComm(void) {}

/********************************************************************************
 * Function: ~SerialComm
 * Description: Class destructor
 ********************************************************************************/
SerialComm::~SerialComm(void) {}

/********************************************************************************
 * Function: start_uart_comm
 * Description: Configure and make available the serial port to be used for
 *              UART or TCP comms.
 ********************************************************************************/
bool SerialComm::start_uart_comm(void)
{
#ifdef USE_UART
    // Open the uart port
    serial_port = open(SERIAL_PORT, O_RDWR);

    if (serial_port < 0)
    {
        fprintf(stderr, "Error starting serial comms\n");
        return false;
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

    uart_fd = serial_port;

    return true;

#elif USE_TCP
    // Create a socket for the TCP connection
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0)
    {
        fprintf(stderr, "Error creating socket\n");
        return false;
    }

    // Configure the address for the TCP connection
    sim_addr.sin_family = AF_INET;
    sim_addr.sin_addr.s_addr = inet_addr(SIM_IP);
    sim_addr.sin_port = htons(SIM_PORT);

    // Connect to the SITL simulation
    if (connect(sock, (struct sockaddr *)&sim_addr, sizeof(sim_addr)) < 0)
    {
        fprintf(stderr, "Error connecting to SITL\n");
        return false;
    }

    // Set the socket to non-blocking mode
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    tcp_socket_fd = sock;                                                  // Ensure the global file descriptor is set
    PrintPass::c_printf("tcp_socket_fd initialized: %d\n", tcp_socket_fd); // Debug log

    return true;

#else
#error "Please define USE_UART or USE_TCP."
#endif
}

/********************************************************************************
 * Function: send_uart
 * Description: Send a buffer of bytes on the uart serial port.
 ********************************************************************************/
void SerialComm::write_uart(mavlink_message_t &msg)
{
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = 0;

    offset_buffer(buffer, len, msg);

#ifdef USE_UART
    // Write to serial port
    uint16_t n = write(serial_port, buffer, len);

    if (MAVLINK_MAX_PACKET_LEN < len)
    {
        fprintf(stderr, "Buffer too large\n");
    }

    if (n < 0)
    {
        fprintf(stderr, "Error writing to serial port\n");
    }

    clear_buffer(buffer, len);

#elif USE_TCP
    // Write to serial port
    uint16_t n = sendto(sock, buffer, len, 0, (struct sockaddr *)&sim_addr, sizeof(sim_addr));

    if (MAVLINK_MAX_PACKET_LEN < len)
    {
        fprintf(stderr, "Buffer too large\n");
    }

    if (n < 0)
    {
        fprintf(stderr, "Error writing to serial port\n");
    }

    clear_buffer(buffer, len);
#else
#error "Please define USE_UART or USE_TCP."
#endif
}

/********************************************************************************
 * Function: read_uart
 * Description: Read some bytes from the uart serial port.
 ********************************************************************************/
uint8_t SerialComm::read_uart(void)
{
#if USE_UART
    uint8_t byte;

    if (read(serial_port, &byte, 1) == -1)
    {
        fprintf(stderr, "Error reading from serial port\n");
    }

    return byte;

#elif USE_TCP
    uint8_t byte;

    ssize_t n = recv(sock, &byte, sizeof(byte), 0);
    if (n < 0)
    {
        fprintf(stderr, "Error reading from TCP port\n");
    }

    return byte;
#else
#error "Please define USE_UART or USE_TCP."
#endif
}

/********************************************************************************
 * Function: stop_uart_comm
 * Description: Close the serial port so it is no longer able to transmit.
 ********************************************************************************/
void SerialComm::stop_uart_comm(void)
{
#if USE_UART
    close(serial_port);
#elif USE_TCP
    close(sock);
#else
#error "Please define USE_UART or USE_TCP."
#endif
}

/********************************************************************************
 * Function: bytes_available
 * Description: Close the serial port so it is no longer able to transmit.
 ********************************************************************************/
int SerialComm::bytes_available(void)
{
    int bytes;

#if USE_UART

    if (ioctl(uart_fd, FIONREAD, &bytes) == -1)
    {
        perror("ioctl");
        return -1;
    }
    return bytes;
#elif USE_TCP

    if (ioctl(tcp_socket_fd, FIONREAD, &bytes) == -1)
    {
        PrintPass::c_printf("ioctl\n");
        return -1;
    }
    return bytes;
#else
#error "Please define USE_UART or USE_TCP."
#endif
}

/********************************************************************************
 * Function: offset_buffer
 * Description: Offset the buffer by the its length to avoid overlapping buffers.
 ********************************************************************************/
void SerialComm::offset_buffer(uint8_t *buffer, uint16_t &len, mavlink_message_t &msg)
{
    len = len + mavlink_msg_to_send_buffer(&buffer[len], &msg);
}

/********************************************************************************
 * Function: clear_buffer
 * Description: Clear the buffer
 ********************************************************************************/
void SerialComm::clear_buffer(uint8_t *buffer, uint16_t len)
{
    memset(buffer, 0, len);
}
