#pragma once

/********************************************************************************
 * @file    scheduler.h
 * @author  Cameron Rose
 * @date    6/7/2023
 ********************************************************************************/
#ifndef SCHEDULER_H
#define SCHEDULER_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include <signal.h> // For siginfo_t, sigaction, and sigevent

/********************************************************************************
 * Imported objects
 ********************************************************************************/

/********************************************************************************
 * Exported objects
 ********************************************************************************/
extern timer_t timerID;
extern struct sigaction sa_timer;
extern struct sigevent sev;
extern struct itimerspec its;
extern bool stopProgram;

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
void initialize(void);
void task_25ms(int sig, siginfo_t *si, void *uc);
void stopHandler(int sig);
void startTask_25ms(void);
void stopTask_25ms(void);
void setupTask_25ms(void);

#endif // SCHEDULER_H
