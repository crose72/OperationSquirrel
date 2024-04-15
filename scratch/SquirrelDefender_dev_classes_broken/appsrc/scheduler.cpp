/********************************************************************************
 * @file    scheduler.cpp
 * @author  Cameron Rose
 * @date    12/27/2023
 * @brief   Create the task(s) that run at the specified rates.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
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
std::mutex mutex;
timer_t timerID;
struct sigaction sa;
struct sigevent sev;
struct itimerspec its;
bool stopProgram = false;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: setupTask_25ms
 * Description: Create the timer.
 ********************************************************************************/
void setupTask_25ms(void)
{
    // Set up the signal handler for stopping the program
    sa.sa_flags = SA_SIGINFO;
    sa.sa_sigaction = task_25ms;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGRTMIN, &sa, nullptr);

    // Set up the signal handler for the timer
    sa.sa_flags = SA_SIGINFO;
    sa.sa_sigaction = task_25ms;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGRTMIN, &sa, nullptr);

    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_signo = SIGRTMIN;
    sev.sigev_value.sival_ptr = &timerID;

    timer_create(CLOCK_REALTIME, &sev, &timerID);

    its.it_interval.tv_sec = 0;
    its.it_interval.tv_nsec = 25e6; // nanoseconds per period for 40Hz
    its.it_value = its.it_interval;
}

/********************************************************************************
 * Function: startTask_25ms
 * Description: Start the 25ms task.
 ********************************************************************************/
void startTask_25ms(void)
{
    timer_settime(timerID, 0, &its, nullptr);
}

/********************************************************************************
 * Function: stopTask_25ms
 * Description: Stop the 25ms task.
 ********************************************************************************/
void stopTask_25ms(void)
{
    timer_delete(timerID);
}

/********************************************************************************
 * Function: stopHandler
 * Description: If this is executed then the main while loop stops.
 ********************************************************************************/
void stopHandler(int sig)
{
    stopProgram = true;
}

/********************************************************************************
 * Function: task_25ms
 * Description: Function to handle timer interrupt at 25ms (40Hz).  This periodic
 *              function runs the main code at the specified rate.
 ********************************************************************************/
void task_25ms(int sig, siginfo_t* si, void* uc)
{

}