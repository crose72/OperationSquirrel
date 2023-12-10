#include "interrupt_handler.h"

uint32_t val1 = 0;
uint32_t val2 = 0;

void incrementVal1(void)
{
    val1 = val1 + 1;
}

void incrementVal2(void)
{
    val2 = val2 + 2;
}

void timerHandler1(int sig, siginfo_t* si, void* uc)
{
    incrementVal1();
    printf("Timer 1: %d\n", val1);
}

void timerHandler2(int sig, siginfo_t* si, void* uc)
{
    incrementVal2();
    printf("Timer 2: %d\n", val2);
}

void createTimerHandler(TimerData* timerData, void (*handler)(int, siginfo_t*, void*))
{
    timerData->sa.sa_flags = SA_SIGINFO;
    timerData->sa.sa_sigaction = handler;
    sigemptyset(&timerData->sa.sa_mask);
    sigaction(SIGRTMIN + timerData->timerIntID, &timerData->sa, nullptr);
}

void createTimer(TimerData* timerData)
{
    // Create the timer
    timerData->sev.sigev_notify = SIGEV_SIGNAL;
    timerData->sev.sigev_signo = SIGRTMIN + timerData->timerIntID;
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
    timer_settime((timer_t)(timerData->timerID), 0, &timerData->its, nullptr);
}

void deleteTimer(TimerData* timerData)
{
    timer_delete((timer_t)(timerData->timerID));
}

void initializeTimers(void)
{
    // Create and initialize Timer 1
    TimerData* timer1; // = (TimerData*)malloc(sizeof(TimerData));
    timer1->timerID = (timer_t)0;
    timer1->timerIntID = (int)0;
    createTimerHandler(timer1, timerHandler1);
    createTimer(timer1);
    configureTimer(timer1, 100); // 100 ms interval
    startTimer(timer1);

    // Create and initialize Timer 2
    TimerData* timer2; // = (TimerData*)malloc(sizeof(TimerData));
    timer2->timerID = (timer_t)1;
    timer2->timerIntID = (int)1;
    createTimerHandler(timer2, timerHandler2);
    createTimer(timer2);
    configureTimer(timer2, 500); // 500 ms interval
    startTimer(timer2);

    // Don't forget to free the memory after using the timers
    /*deleteTimer(timer1);
    deleteTimer(timer2);
    free(timer1);
    free(timer2);*/
}

