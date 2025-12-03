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
#include "common_inc.h"
#include "scheduler.h"
#include "datalog.h"
#include "video_io.h"
#include "system_controller.h"
#include "mav_data_hub.h"
#include "mav_utils.h"
#include "vehicle_controller.h"
#include "target_detection.h"
#include "target_tracking.h"
#include "target_localization.h"
#include "velocity_controller.h"
#include "time_calc.h"
#include "timer.h"
#include "velocity_controller.h"
#include <mutex>
#include <signal.h>
#include <chrono>
#include <thread>

#ifdef BLD_JETSON_B01

#include "status_io.h"

#endif // BLD_JETSON_B01

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
bool g_system_init;
Timer main_loop(std::chrono::milliseconds(60));
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
        !TargetDetection::init() ||
        !TargetTracking::init() ||
        !TargetLocalization::init() ||
        !VelocityController::init())
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

    g_system_init = true;

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
    TargetDetection::loop();
    TargetTracking::loop();
    TargetLocalization::loop();
    VelocityController::loop();
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

    TargetTracking::shutdown();
    TargetLocalization::shutdown();
    VelocityController::shutdown();
    VelocityController::shutdown();
    Video::shutdown();
    TargetDetection::shutdown();

#endif // ENABLE_CV
}
