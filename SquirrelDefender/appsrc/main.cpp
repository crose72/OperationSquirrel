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
#include <mutex>
#include "scheduler.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/
bool g_stop_program;
bool g_use_video_playback;
std::string input_video_path;

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
        g_stop_program = true;
        Print::c_fprintf("received SIGINT\n");
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
        Print::c_fprintf("can't catch SIGINT");
    }
}

/********************************************************************************
 * Function: main
 * Description: Entry point for the program.  Runs the main loop.
 ********************************************************************************/
int main(int argc, char** argv) 
{
    input_video_path = "";
    g_use_video_playback = false;
    g_stop_program = false;
    
    if (argc > 1)
    {
        input_video_path = argv[1];
        g_use_video_playback = true;
    }
    
    attach_sig_handler();

    if (Scheduler::init() != 0)
    {
        return Scheduler::init();
    }

    while (!g_stop_program)
    {
        Scheduler::loop();
    }

    Scheduler::shutdown();

    return 0;
}
