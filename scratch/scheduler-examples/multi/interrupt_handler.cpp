#include "interrupt_handler.h"

void timerHandler1(int sig, siginfo_t* si, void* uc)
{
    TimerData* timerData = reinterpret_cast<TimerData*>(si->si_value.sival_ptr);
    // Perform tasks here that need to run at the desired frequency for Timer 1
    // For example:
    printf("Timer 1: %d\n", timerData->timerID);
}

void timerHandler2(int sig, siginfo_t* si, void* uc)
{
    TimerData* timerData = reinterpret_cast<TimerData*>(si->si_value.sival_ptr);
    // Perform tasks here that need to run at the desired frequency for Timer 2
    // For example:
    printf("Timer 2: %d\n", timerData->timerID);
}

void createTimerHandler1(TimerData* timerData)
{
    timerData->sa.sa_flags = SA_SIGINFO;
    timerData->sa.sa_sigaction = timerHandler1;
    sigemptyset(&timerData->sa.sa_mask);
    sigaction(SIGRTMIN, &timerData->sa, nullptr);
}

void createTimerHandler2(TimerData* timerData)
{
    timerData->sa.sa_flags = SA_SIGINFO;
    timerData->sa.sa_sigaction = timerHandler2;
    sigemptyset(&timerData->sa.sa_mask);
    sigaction(SIGRTMIN, &timerData->sa, nullptr);
}

void createTimer(TimerData* timerData)
{
    // Create the timer
    timerData->sev.sigev_notify = SIGEV_SIGNAL;
    timerData->sev.sigev_signo = SIGRTMIN;
    timerData->sev.sigev_value.sival_ptr = &timerData->timerID;
    timer_create(CLOCK_REALTIME, &timerData->sev, &timerData->timerID);
}

void configureTimer(TimerData* timerData, long intervalMs)
{
    // Configure the timer to fire at the specified interval
    timerData->its.it_value.tv_sec = intervalMs / 1000;
    timerData->its.it_value.tv_nsec = (intervalMs % 1000) * 1000000; // Convert to nanoseconds
    timerData->its.it_interval = timerData->its.it_value; // Repeating interval is the same as the initial interval
}

void startTimer(TimerData* timerData)
{
    // Start the timer
    timer_settime(timerData->timerID, 0, &timerData->its, nullptr);
}

void deleteTimer(TimerData* timerData)
{
    timer_delete(timerData->timerID);
}

void initializeTimers(void)
{
    // Create and initialize Timer 1
    TimerData timer1;
    createTimerHandler1(&timer1);
    createTimer(&timer1);
    configureTimer(&timer1, 100); // 100 ms interval
    startTimer(&timer1);

    // Create and initialize Timer 2
    TimerData timer2;
    createTimerHandler2(&timer2);
    createTimer(&timer2);
    configureTimer(&timer2, 500); // 500 ms interval
    startTimer(&timer2);
}
