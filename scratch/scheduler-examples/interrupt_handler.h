#ifndef INTERRUPT_HANDLER_H
#define INTERRUPT_HANDLER_H

#include <signal.h>
#include <time.h>
#include <vector>
#include <iostream>

// Custom structure to hold timer-related variables for each timer
typedef struct {
    timer_t timerID;
    int timerIntID;
    struct sigaction sa;
    struct sigevent sev;
    struct itimerspec its;
} TimerData;

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
