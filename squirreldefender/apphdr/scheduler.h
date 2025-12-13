/********************************************************************************
 * @file    scheduler.h
 * @author  Cameron Rose
 * @date    3/12/2025
 * @brief   Top-level system scheduler responsible for initializing modules and
 *          running the main loop execution order. Provides init/loop/shutdown
 *          entry points used by the main application.
 ********************************************************************************/
#ifndef SCHEDULER_H
#define SCHEDULER_H

/********************************************************************************
 * Includes
 ********************************************************************************/

/********************************************************************************
 * Exported objects
 ********************************************************************************/
extern bool g_system_init;

/********************************************************************************
 * Function prototypes
 ********************************************************************************/
class Scheduler
{
public:
    Scheduler();
    ~Scheduler();

    static int init(void);
    static void loop(void);
    static void shutdown(void);

private:
};

#endif // SCHEDULER_H
