/********************************************************************************
 * @file    time_calc.cpp
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Calculate program run time and main loop rate.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include <chrono>
#include <cmath>

#include "common_inc.h"
#include "time_calc.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/

// Runtime elapsed timing (steady_clock)
float g_app_time_s;     // Elapsed app time [s], truncated to ms
float g_app_dt;         // Delta time between loops [s]
uint64_t g_app_time_ns; // Elapsed app time since start [ns]
float app_time_s_prv;   // Previous loop timestamp [s] for computing dt

// Epoch timing (wall clock)
uint64_t g_app_epoch_ns; // Current Unix epoch time [ns]
uint64_t g_offset_ns;    // steady_clock → epoch offset [ns]

// App start baseline (steady_clock)
static std::chrono::steady_clock::time_point g_start_steady; // Steady-clock baseline at init

// Loop initialization flag
bool g_app_first_loop; // True on the first loop iteration

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/
void calc_app_runtime(void);
uint64_t now_epoch_ns();   // monotonic, in ns since Unix epoch
uint64_t now_elapsed_ns(); // ns since app start (monotonic)

static inline uint64_t steady_now_ns()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}
static inline uint64_t system_now_ns()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

uint64_t now_epoch_ns()
{
    // monotonic steady time + fixed offset → epoch ns
    return g_offset_ns + steady_now_ns();
}

uint64_t now_elapsed_ns()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now() - g_start_steady)
        .count();
}

static inline uint64_t epoch_now_ns()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

/********************************************************************************
 * Function: calc_app_runtime
 * Description: Calculate how long the program has been running.  Use
 *              steady_clock monotomic timing.
 ********************************************************************************/
void calc_app_runtime(void)
{
    if (g_app_first_loop)
    {
        g_app_first_loop = false;
        app_time_s_prv = 0.0f;
        g_app_time_s = 0.0f;
        g_app_dt = 0.0f;
        return;
    }

    // elapsed (ns) since app start using the same baseline as now_elapsed_ns()
    const uint64_t elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    std::chrono::steady_clock::now() - g_start_steady)
                                    .count();

    // convert to seconds (float) for your existing API, with ms precision
    const float app_elapsed_time_tmp = static_cast<float>(elapsed_ns) * 1e-9f;

    g_app_dt = app_elapsed_time_tmp - app_time_s_prv;

    // Truncate to three decimal places (ms)
    g_app_time_s = std::floor(app_elapsed_time_tmp * 1000.0f) / 1000.0f;
    app_time_s_prv = g_app_time_s;
}

/********************************************************************************
 * Function: Time
 * Description: Class constructor
 ********************************************************************************/
Time::Time(void) {}

/********************************************************************************
 * Function: ~Time
 * Description: Class destructor
 ********************************************************************************/
Time::~Time(void) {}

/********************************************************************************
 * Function: init
 * Description: Initialize time calculation.
 ********************************************************************************/
bool Time::init(void)
{
    g_app_first_loop = true;
    g_start_steady = std::chrono::steady_clock::now();

    // compute fixed offset so steady→epoch is monotonic but epoch-referenced
    g_offset_ns = system_now_ns() - steady_now_ns();
    return true;
}

/********************************************************************************
 * Function: loop
 * Description: Main time calculation loop.
 ********************************************************************************/
void Time::loop(void)
{
    calc_app_runtime();
    g_app_time_ns = now_elapsed_ns();
    g_app_epoch_ns = epoch_now_ns();
}

/********************************************************************************
 * Function: shutdown
 * Description: Shutdown time calculation.
 ********************************************************************************/
void Time::shutdown(void)
{
}