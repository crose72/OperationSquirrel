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

#ifdef BLD_JETSON_B01

#include "status_io.h"

#endif // BLD_JETSON_B01

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/
bool stop_program;

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
        stop_program = true;
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
int main(void)
{

    attach_sig_handler();
    stop_program = false;

    if (Scheduler::init() != 0)
    {
        return Scheduler::init();
    }

#ifdef BLD_JETSON_B01

    while (!stop_program && !g_save_button_press) // todo: figure buttons w docker (jetson io/jumper)

#else

    while (!stop_program && !g_manual_override_land)

#endif // BLD_JETSON_B01

    {
        Scheduler::loop();
    }

    Scheduler::shutdown();

    return 0;
}
