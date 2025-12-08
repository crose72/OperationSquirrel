/********************************************************************************
 * @file    main.cpp
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Main entry point for the program, contains the loop functions for
            each software component.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "scheduler.h"
#include "global_objects.h"
#include <spdlog/spdlog.h>
#include <mutex>
#include <signal.h>
#include <fenv.h>

#pragma STDC FENV_ACCESS ON

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: sig_handler
 * Description: Stop the program if Crl+C is entered in the terminal.
 ********************************************************************************/
void sig_handler(int signo)
{
    if (signo == SIGINT)
    {
        g_app_stop = true;
        spdlog::info("received SIGINT\n");
    }
}

/********************************************************************************
 * Function: attach_sig_handler
 * Description: Attach sig handler to enable program termination by Crl+C.
 ********************************************************************************/
void attach_sig_handler(void)
{
    if (signal(SIGINT, sig_handler) == SIG_ERR)
    {
        spdlog::error("can't catch SIGINT");
    }
    if (signal(SIGTERM, sig_handler) == SIG_ERR)
    {
        spdlog::error("Can't catch SIGTERM");
    }
}

/********************************************************************************
 * Function: main
 * Description: Entry point for the program.  Runs the main loop.
 ********************************************************************************/
int main(int argc, char **argv)
{
    // Optional floating-point exceptions for debugging
    // to catch NaN, Inf, and other errors (DO NOT USE FOR REAL FLIGHT)
    // feenableexcept(FE_INVALID | FE_DIVBYZERO | FE_OVERFLOW);

    // Default path is to use a live camera feed
    g_app_video_input_path = "";
    g_app_use_video_playback = false;
    g_app_stop = false;

    if (argc > 1)
    {
        g_app_video_input_path = argv[1];
        g_app_use_video_playback = true;
    }

    // Allow the program to complete by pressing Ctrl+C in terminal
    attach_sig_handler();

    // Initialize the software modules
    if (Scheduler::init() != 0)
    {
        return Scheduler::init();
    }

    // Main scheduler loop
    while (!g_app_stop)
    {
        Scheduler::loop();
    }

    // Graceful shutdown
    Scheduler::shutdown();

    // Ensures logs and files are flushed to disk
    // (save everything now so files are not lost/corrupt)
    ::sync();

    return 0;
}
