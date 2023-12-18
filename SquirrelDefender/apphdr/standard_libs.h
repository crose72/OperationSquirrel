#ifndef STANDARD_LIBS
#define STANDARD_LIBS

// Standard headers
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

#ifdef USE_TCP
    #include <sys/socket.h>
    #include <arpa/inet.h>
    #include <netinet/in.h>
#endif

// Customer headers for global objects
#include "global_objects.h"
#include "global_calibrations.h"
#include "global_types.h"

#endif // STANDARD_LIBS