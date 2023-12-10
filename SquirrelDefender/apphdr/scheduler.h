#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "standard_libs.h"

extern std::mutex mutex;
extern timer_t timerID;
extern struct sigaction sa_timer;
extern struct sigevent sev;
extern struct itimerspec its;

extern bool stopProgram;

void initialize(void);
void task_25ms(int sig, siginfo_t* si, void* uc);
void stopHandler(int sig);
void startTask_25ms(void);
void stopTask_25ms(void);
void setupTask_25ms(void);

#endif // SCHEDULER_H
