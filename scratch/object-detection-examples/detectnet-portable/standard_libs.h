#ifndef STANDARD_LIBS
#define STANDARD_LIBS

// Standard headers
#include <iostream>
#include <signal.h>
#include <chrono>
#include <ctime>
#include <time.h>
#include <mutex>
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

#endif // STANDARD_LIBS