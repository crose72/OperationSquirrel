/********************************************************************************
 * @file    scheduler.cpp
 * @author  Cameron Rose
 * @date    3/12/2025
 * @brief   The scheduler handles the scheduling and priority of all code running
 *          in a loop.
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
std::mutex scheduler_mutex;
bool g_controller_initialiazed;
Timer main_loop(std::chrono::milliseconds(25));
Timer timer1;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function:
 * Description:
 ********************************************************************************/

/********************************************************************************
 * Function: Scheduler
 * Description: Scheduler class constructor.
 ********************************************************************************/
Scheduler::Scheduler(void) {};

/********************************************************************************
 * Function: ~Scheduler
 * Description: Scheduler class destructor.
 ********************************************************************************/
Scheduler::~Scheduler(void) {};

/********************************************************************************
 * Function: init
 * Description: Initialize all planner variables.  Run once at the start
 *              of the program.
 ********************************************************************************/
int Scheduler::init(void)
{

#ifdef ENABLE_CV

    if (!Video::init() ||
        !Detection::init() ||
        !Tracking::init() ||
        !Localize::init() ||
        !PathPlanner::init())
    {
        return 1;
    }

#endif // ENABLE_CV

    if (!Time::init() ||
        !MavMsg::init() ||
        !DataLogger::init() ||
        !VehicleController::init() ||
        !SystemController::init())
    {
        return 1;
    }

    g_controller_initialiazed = true;

    return 0;
}

/********************************************************************************
 * Function: loop
 * Description: Function to run every loop.
 ********************************************************************************/
void Scheduler::loop(void)
{
    std::lock_guard<std::mutex> lock(scheduler_mutex);
    main_loop.start();
    SystemController::loop();
    MavMsg::loop();

#ifdef ENABLE_CV

    Video::in_loop();
    Detection::loop();
    Tracking::loop();
    Localize::loop();
    PathPlanner::loop();
    Video::out_loop();

#endif // ENABLE_CV

    VehicleController::loop();
    Time::loop(); // Keep this before the datalogger - else MCAP breaks
    DataLogger::loop();
    main_loop.stop();
    main_loop.wait();
}

/********************************************************************************
 * Function: shutdown
 * Description: Function to clean up planner at the end of the program.
 ********************************************************************************/
void Scheduler::shutdown(void)
{
    SystemController::shutdown();
    VehicleController::shutdown();
    MavMsg::shutdown();
    DataLogger::shutdown();
    Time::shutdown();

#ifdef ENABLE_CV

    Tracking::shutdown();
    Localize::shutdown();
    PathPlanner::shutdown();
    PathPlanner::shutdown();
    Video::shutdown();
    Detection::shutdown();

#endif // ENABLE_CV
}
