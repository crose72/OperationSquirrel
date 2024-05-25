#pragma once

/********************************************************************************
 * @file    common_inc.h
 * @author  Cameron Rose
 * @date    12/27/2023
 ********************************************************************************/
#ifndef COMMON_INC
#define COMMON_INC

/********************************************************************************
 * Includes
 ********************************************************************************/
#include <mavlink.h>
#include <iostream>
#include <signal.h>
#include <chrono>
#include <ctime>
#include <cmath>
#include <time.h>
#include <mutex>
#include <common.h>
#include <thread>
#include <stdint.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>

// Serial port stuff
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <cstdio>  // for perror
#include <cstdint> // for uint8_t, uint16_t, etc.

// Custom includes for global objects
#include "global_objects.h"
#include "global_calibrations.h"
#include "global_types.h"

// Conditional includes
#ifdef USE_TCP
    #include <sys/socket.h>
    #include <arpa/inet.h>
    #include <netinet/in.h>
#endif

#endif // COMMON_INC
