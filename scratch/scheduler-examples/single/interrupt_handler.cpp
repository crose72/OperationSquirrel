#include "interrupt_handler.h"

// Timer variables
timer_t timerID;

// Create a POSIX timer
struct sigevent sev;
struct itimerspec its;

// Set up the timer handler
struct sigaction sa;
uint32_t val = 0;

void incrementVal(void)
{
    val = val + 1;
}

// Timer handler function
void timerHandler(int sig, siginfo_t* si, void* uc)
{
    incrementVal();
    printf("%d\n", val);
}

void createTimerHandler(void)
{
    sa.sa_flags = SA_SIGINFO;
    sa.sa_sigaction = timerHandler;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGRTMIN, &sa, nullptr);
}

void createTimer(void)
{
    // Create the timer
    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_signo = SIGRTMIN;
    sev.sigev_value.sival_ptr = &timerID;
    timer_create(CLOCK_REALTIME, &sev, &timerID);
}

void configureTimer(void)
{
    // Configure the timer to fire every 100 milliseconds (10 Hz)
    its.it_value.tv_nsec = 100000000; // X million nanoseconds (X * 10e-3 milliseconds)
    its.it_interval.tv_nsec = 100000000; // X million nanoseconds (X * 10e-3 milliseconds)
}

void startTimer(void)
{
    // Start the timer
    timer_settime(timerID, 0, &its, nullptr);
}

void deleteTimer(void)
{
    timer_delete(timerID);
}

void initializeTimer(void)
{
    timerID = 0;
    createTimerHandler();
    createTimer();
    configureTimer();
    startTimer();
}
