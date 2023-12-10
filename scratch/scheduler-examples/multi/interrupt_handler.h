#ifndef INTERRUPT_HANDLER_H
#define INTERRUPT_HANDLER_H

#include <signal.h>
#include <time.h>
#include <vector>
#include <iostream>

// Custom structure to hold timer-related variables for each timer
struct TimerData {
    timer_t timerID;
    struct sigevent sev;
    struct itimerspec its;
    struct sigaction sa;
};

void createTimer(TimerData* timerData);
void configureTimer(TimerData* timerData, long intervalMs);
void startTimer(TimerData* timerData);
void deleteTimer(TimerData* timerData);
void initializeTimers(void);
void timerHandler1(int sig, siginfo_t* si, void* uc);
void timerHandler2(int sig, siginfo_t* si, void* uc);
void createTimerHandler1(TimerData* timerData);
void createTimerHandler2(TimerData* timerData);

#endif // INTERRUPT_HANDLER_H
