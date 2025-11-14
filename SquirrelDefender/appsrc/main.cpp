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
    feenableexcept(FE_INVALID | FE_DIVBYZERO | FE_OVERFLOW);

    g_app_video_input_path = "";
    g_app_use_video_playback = false;
    g_app_stop = false;

    if (argc > 1)
    {
        g_app_video_input_path = argv[1];
        g_app_use_video_playback = true;
    }

    attach_sig_handler();

    if (Scheduler::init() != 0)
    {
        return Scheduler::init();
    }

    while (!g_app_stop)
    {
        Scheduler::loop();
    }

    Scheduler::shutdown();
    ::sync();

    return 0;
}
