/********************************************************************************
 * @file    scheduler.cpp
 * @author  Cameron Rose
 * @date    3/12/2025
 * @brief   The scheduler handles the initialization, scheduling and priority
 *          of all code in the program.
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

#ifdef BLD_JETSON_B01

#include "status_io.h"

#endif // BLD_JETSON_B01

#include <mutex>
#include <signal.h>
#include <chrono>
#include <thread>

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
 * Description: Initialize all software components.  Run once at the start
 *              of the program.  Returns 0 if successful, returns non-zero
 *              number otherwise to terminate execution of the program.
 ********************************************************************************/
int Scheduler::init(void)
{
#ifdef ENABLE_CV

    /* Vision-related modules.
     *
     * These are initialized only when camera/video support is enabled.
     * They depend on OpenCV, CUDA, TensorRT, and GPU memory management.
     * The order here matches the order in which these modules run inside
     * the main loop.
     */
    if (!Video::init() ||
        !TargetDetection::init() ||
        !TargetTracking::init() ||
        !TargetLocalization::init())
    {
        return 1;
    }

#endif // ENABLE_CV

    /* Core system modules.
     *
     * These modules do not rely on GPU libraries and are always required,
     * even when computer vision is disabled. They use standard C/C++ or
     * lightweight embedded code.
     */
    if (!Time::init() ||
        !MavMsg::init() ||
        !DataLogger::init() ||
        !VelocityController::init() ||
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
 * Description: Run all software components.
 ********************************************************************************/
void Scheduler::loop(void)
{
    std::lock_guard<std::mutex> lock(scheduler_mutex);
    main_loop.start();

    /* High-level system state machine.
     * Handles arming, failsafes, and mode transitions.
     */
    SystemController::loop();

    // Handle incoming/outgoing MAVLink messages
    MavMsg::loop();

#ifdef ENABLE_CV

    /* Vision pipeline:
     *   - read frame (camera or playback)
     *   - run detector
     *   - run tracker
     *   - estimate 3D target position
     *   - compute target-relative velocity commands
     *   - output annotated frame
     */
    Video::in_loop();
    TargetDetection::loop();
    TargetTracking::loop();
    TargetLocalization::loop();
    Video::out_loop();

#endif // ENABLE_CV

    // Calculate vehicle velocity commands
    VelocityController::loop();

    // Apply vehicle controls (velocity, takeoff/land, mode changes, etc.)
    VehicleController::loop();

    // Time update must run before the logger to preserve ordering in MCAP
    Time::loop();

    // Save telemetry, detections, and visualizations
    DataLogger::loop();
    main_loop.stop();
    main_loop.wait(); // Enforce loop rate
}

/********************************************************************************
 * Function: shutdown
 * Description: Final tasks to ensure graceful shutdown.
 ********************************************************************************/
void Scheduler::shutdown(void)
{
    SystemController::shutdown();
    VehicleController::shutdown();

    // Close MAVLink connections (send final commands before calling this)
    MavMsg::shutdown();

    DataLogger::shutdown();
    Time::shutdown();

#ifdef ENABLE_CV

    TargetTracking::shutdown();
    TargetLocalization::shutdown();
    VelocityController::shutdown();
    Video::shutdown();
    TargetDetection::shutdown();

#endif // ENABLE_CV
}
