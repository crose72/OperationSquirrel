#ifndef INTERRUPT_HANDLER_H
#define INTERRUPT_HANDLER_H

#include <signal.h>
#include <time.h>
#include <iostream>

void createTimerHandler(void);
void createTimer(void);
void configure_timer(void);
void startTimer(void);
void deleteTimer(void);
void initializeTimer(void);

#endif // INTERRUPT_HANDLER_H
