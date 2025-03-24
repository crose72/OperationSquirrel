/********************************************************************************
 * @file    scheduler.h
 * @author  Cameron Rose
 * @date    3/12/2025
 ********************************************************************************/
#ifndef SCHEDULER_H
#define SCHEDULER_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include <mutex>
#include <signal.h>
#include <chrono>
#include <thread>
#include "common_inc.h"
#include "datalog.h"
#include "video_io.h"
#include "system_controller.h"
#include "mav_data_hub.h"
#include "mav_utils.h"
#include "vehicle_controller.h"
#include "detect_target.h"
#include "track_target.h"
#include "localize_target.h"
#include "follow_target.h"
#include "time_calc.h"
#include "timer.h"
#include "path_planner.h"

#ifdef BLD_JETSON_B01

#include "status_io.h"

#endif // BLD_JETSON_B01

/********************************************************************************
 * Imported objects
 ********************************************************************************/

/********************************************************************************
 * Exported objects
 ********************************************************************************/

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
