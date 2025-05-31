/********************************************************************************
 * @file    mav_serial.cpp
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Utilities for sending and receiving mavlink messages over UART or TCP
 *          for simulated and real serial connections.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "mav_serial.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/
#ifdef BLD_LINUX_SERIAL

#define SERIAL_PORT "/dev/ttyTHS1" // Default device for UART on jetson nano - modify if needed
#define BAUD_RATE B115200

#elif defined(BLD_WIN_SERIAL)

#define SERIAL_PORT "COM3"  // Modify based on your USB port (e.g., COM3, COM4, etc.)
#define BAUD_RATE CBR_57600 // 57600 for SiK radio, CBR_115200 typically

#elif defined(BLD_WSL_TCP) || defined(BLD_WIN_TCP)

/* Sim port/address for linux or windows */
#define SIM_IP "127.0.0.1" // IP address for SITL simulation
#define SIM_PORT 5762      // Port number for SITL

#else

#error "Please define the intended build target."

#endif // BLD_LINUX_SERIAL

/********************************************************************************
 * Object definitions
 ********************************************************************************/
#ifdef BLD_LINUX_SERIAL

int uart_fd; // UART file descriptor for Linux
int serial_port = 0;

#elif defined(BLD_WIN_SERIAL)

HANDLE hComm; // Handle for COM port on Windows

#elif defined(BLD_WSL_TCP) || defined(BLD_WIN_TCP)

SOCKET tcp_socket_fd; // TCP socket descriptor
struct sockaddr_in sim_addr;
int sock;

#else

#error "Please define the intended build target."

#endif // BLD_LINUX_SERIAL

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: Serial
 * Description: Class constructor
 ********************************************************************************/
MavSerial::MavSerial(void) {}

/********************************************************************************
 * Function: ~Serial
 * Description: Class destructor
 ********************************************************************************/
MavSerial::~MavSerial(void) {}

/********************************************************************************
 * Function: start_mav_comm
 * Description: Configure and make available the serial port for UART or TCP comms.
 ********************************************************************************/
bool MavSerial::start_mav_comm(void)
{
#ifdef BLD_LINUX_SERIAL

    // Open the UART port for Linux
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

#elif defined(BLD_WIN_SERIAL)

    // Open the COM port
    hComm = CreateFile(SERIAL_PORT, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (hComm == INVALID_HANDLE_VALUE)
    {
        DWORD errCode = GetLastError();
        fprintf(stderr, "Error opening COM port. Error code: %lu\n", errCode);
        exit(EXIT_FAILURE); // Terminate the program with failure status
        return false;
    }

    // Configure the COM port
    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(hComm, &dcbSerialParams))
    {
        fprintf(stderr, "Error getting COM port state\n");
        exit(EXIT_FAILURE); // Terminate the program with failure status
        return false;
    }

    dcbSerialParams.BaudRate = BAUD_RATE;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;

    if (!SetCommState(hComm, &dcbSerialParams))
    {
        fprintf(stderr, "Error setting COM port state\n");
        return false;
    }

    // Set timeouts for the serial port
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 50;         // Maximum time between two bytes, in ms
    timeouts.ReadTotalTimeoutConstant = 50;    // Total timeout constant, in ms
    timeouts.ReadTotalTimeoutMultiplier = 10;  // Per-byte timeout multiplier, in ms
    timeouts.WriteTotalTimeoutConstant = 50;   // Write timeout constant, in ms
    timeouts.WriteTotalTimeoutMultiplier = 10; // Per-byte write timeout multiplier, in ms

    if (!SetCommTimeouts(hComm, &timeouts))
    {
        fprintf(stderr, "Error setting COM port timeouts\n");
        return false;
    }

    return true;

#elif defined(BLD_WSL_TCP) || defined(BLD_WIN_TCP) // Create a TCP connection, most of the logic is common to BLD_WSL_TCP and WIN32

#ifdef BLD_WIN_TCP // Initialize WinSock (Windows only)

    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
    {
        fprintf(stderr, "WSAStartup failed\n");
        return false;
    }

#endif

    // Create a socket for the TCP connection - common for windows and linux
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

#ifdef BLD_WIN_TCP

    u_long mode = 1;
    ioctlsocket(sock, FIONBIO, &mode);

#else // BLD_WSL_TCP

    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

#endif

    tcp_socket_fd = sock;                                     // Ensure the global file descriptor is set
    printf("tcp_socket_fd initialized: %d\n", tcp_socket_fd); // Debug log

    return true;

#else

#error "Please define the intended build target."

#endif // BLD_LINUX_SERIAL
}

/********************************************************************************
 * Function: send_uart
 * Description: Send a buffer of bytes on the UART Serial port or TCP connection.
 ********************************************************************************/
void MavSerial::write_mav_msg(mavlink_message_t &msg)
{
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = 0;

    offset_buffer(buffer, len, msg);

#ifdef BLD_LINUX_SERIAL

    uint16_t n = write(serial_port, buffer, len);
    if (n < 0)
    {
        fprintf(stderr, "Error writing to serial port\n");
    }

#elif defined(BLD_WIN_SERIAL)

    DWORD bytesWritten;
    if (!WriteFile(hComm, buffer, len, &bytesWritten, NULL))
    {
        fprintf(stderr, "Error writing to COM port\n");
    }

#elif defined(BLD_WSL_TCP) || defined(BLD_WIN_TCP) // send over TCP, common to BLD_WSL_TCP and BLD_WIN_TCP

    int n = send(tcp_socket_fd, (const char *)buffer, len, 0);
    if (n == SOCKET_ERROR)
    {
        fprintf(stderr, "Error writing to TCP socket\n");
    }

#else

#error "Please define the intended build target."

#endif // BLD_LINUX_SERIAL

    clear_buffer(buffer, len);
}

/********************************************************************************
 * Function: read_mav_msg
 * Description: Read some bytes from the UART serial port or TCP connection.
 ********************************************************************************/
uint8_t MavSerial::read_mav_msg(void)
{
    uint8_t byte = 0;
    int n = 0; // Declare n to avoid undeclared identifier error

#ifdef BLD_LINUX_SERIAL

    if (read(serial_port, &byte, 1) == -1)
    {
        fprintf(stderr, "Error reading from serial port\n");
    }

#elif defined(BLD_WIN_SERIAL)

    if (hComm == INVALID_HANDLE_VALUE)
    {
        DWORD errCode = GetLastError();
        fprintf(stderr, "Error opening COM port. Error code: %lu\n", errCode);
        return false;
    }

    DWORD bytesRead;
    if (!ReadFile(hComm, &byte, 1, &bytesRead, NULL))
    {
        DWORD errCode = GetLastError();
        fprintf(stderr, "Error reading from COM port. Error code: %lu\n", errCode);
    }

#elif defined(BLD_WSL_TCP) || defined(BLD_WIN_TCP) // read TCP port common to BLD_WSL_TCP and BLD_WIN_TCP

    n = recv(tcp_socket_fd, (char *)&byte, sizeof(byte), 0);
    if (n < 0)
    {
        fprintf(stderr, "Error reading from TCP port\n");
    }

#else

#error "Please define the intended build target."

#endif // BLD_LINUX_SERIAL

    return byte;
}

/********************************************************************************
 * Function: stop_mav_comm
 * Description: Close the Serial port or TCP connection.
 ********************************************************************************/
void MavSerial::stop_mav_comm(void)
{

#ifdef BLD_LINUX_SERIAL

    close(serial_port);

#elif defined(BLD_WIN_SERIAL)

    CloseHandle(hComm);

#elif defined(BLD_WIN_TCP)

    closesocket(tcp_socket_fd);
    WSACleanup();

#elif defined(BLD_WSL_TCP)

    close(tcp_socket_fd);

#else

#error "Please define the intended build target."

#endif // BLD_LINUX_SERIAL

}

/********************************************************************************
 * Function: bytes_available
 * Description: Check how many bytes are available for reading.
 ********************************************************************************/
int MavSerial::bytes_available(void)
{
    int bytes = 0;

#ifdef BLD_LINUX_SERIAL

    if (ioctl(uart_fd, FIONREAD, &bytes) == -1)
    {
        perror("ioctl");
        return -1;
    }

#elif defined(BLD_WIN_SERIAL)

    COMSTAT status;

    DWORD errors;
    ClearCommError(hComm, &errors, &status);
    bytes = status.cbInQue;

#elif defined(BLD_WIN_TCP)

    u_long availableBytes = 0;
    ioctlsocket(tcp_socket_fd, FIONREAD, &availableBytes);
    bytes = (int)availableBytes;

#elif defined(BLD_WSL_TCP) // BLD_WSL_TCP

    if (ioctl(tcp_socket_fd, FIONREAD, &bytes) == -1)
    {
        perror("ioctl");
        return -1;
    }

#else

#error "Please define the intended build target."

#endif // BLD_LINUX_SERIAL

    return bytes;
}

/********************************************************************************
 * Function: offset_buffer
 * Description: Offset the buffer by its length to avoid overlapping buffers.
 ********************************************************************************/
void MavSerial::offset_buffer(uint8_t *buffer, uint16_t &len, mavlink_message_t &msg)
{
    len = len + mavlink_msg_to_send_buffer(&buffer[len], &msg);
}

/********************************************************************************
 * Function: clear_buffer
 * Description: Clear the buffer.
 ********************************************************************************/
void MavSerial::clear_buffer(uint8_t *buffer, uint16_t len)
{
    memset(buffer, 0, len);
}
