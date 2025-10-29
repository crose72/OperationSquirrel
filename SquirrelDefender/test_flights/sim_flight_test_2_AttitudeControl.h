/********************************************************************************
 * @file    sim_flight_test_2_AttitudeControl.h
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   A test to understand the set attitude target message for controlling
 *          the drone.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "mav_data_hub.h"
#include "mav_utils.h"
#include <spdlog/spdlog.h>
#include "pid.h"
#include "param_reader.h"

/********************************************************************************
 * Imported objects
 ********************************************************************************/

/********************************************************************************
 * Test flight object definitions
 ********************************************************************************/
float timerVal = 0;
int stage = 0;
float hover_thrust = (float)0.0;
const float dt = 0.025;
PID pid_thrust;
float thrust_cmd;
float Kp_thrust;
float Ki_thrust;
float Kd_thrust;
float w1_thrust;
float alt_error;
bool init = false;
float CLIMB_BOOST_STEP;
float CLIMB_BOOST_MAX;
float CLIMB_BOOST_DECAY;
float ALT_TOL;
float SLOW_ZONE;
float VZ_MIN;
float target_alt;

/********************************************************************************
 * Additional functions
 ********************************************************************************/
void countupTimer(void)
{
    timerVal = timerVal + dt; // printf("Timer: %.3f\n", timerVal);
}

void resetTimer(void)
{
    timerVal = 0;
}

void takeoff(float hover_thrust)
{
    // Ramp up thrust smoothly
    float q[4] = {1, 0, 0, 0}; // flat attitude (no tilt)
    float thrust_body[3] = {0, 0, 0};
    float thrust = 1.0f;

    MavMotion::send_cmd_set_attitude_target(
        SENDER_SYS_ID,
        SENDER_COMP_ID,
        TARGET_SYS_ID,
        TARGET_COMP_ID,
        0b00000111, // ignore rates, use attitude+thrust
        q,
        0, 0, 0,
        hover_thrust,
        thrust_body);
}

void init_flight(void)
{
    ParamReader height_control("../params.json");

    // --- PID Parameters ---
    Kp_thrust = height_control.get_float_param("PID_thrust", "Kp");
    Ki_thrust = height_control.get_float_param("PID_thrust", "Ki");
    Kd_thrust = height_control.get_float_param("PID_thrust", "Kd");
    w1_thrust = height_control.get_float_param("PID_thrust", "w1");

    // --- Hover thrust baseline (from MAVLink param or fallback) ---
    MavCmd::read_param(SENDER_SYS_ID, SENDER_COMP_ID, TARGET_SYS_ID, TARGET_COMP_ID,
                       "MOT_THST_HOVER", -1);

    // --- Mode and arming ---
    MavCmd::set_mode_guided_nogps(SENDER_SYS_ID, SENDER_COMP_ID, TARGET_SYS_ID, TARGET_COMP_ID);
    MavCmd::arm_vehicle(SENDER_SYS_ID, SENDER_COMP_ID, TARGET_SYS_ID, TARGET_COMP_ID);

    // --- Adaptive climb assist parameters ---
    CLIMB_BOOST_STEP = height_control.get_float_param("ClimbAssist", "BoostStep");
    CLIMB_BOOST_MAX = height_control.get_float_param("ClimbAssist", "BoostMax");
    CLIMB_BOOST_DECAY = height_control.get_float_param("ClimbAssist", "BoostDecay");
    ALT_TOL = height_control.get_float_param("ClimbAssist", "AltTol");
    SLOW_ZONE = height_control.get_float_param("ClimbAssist", "SlowZone");
    VZ_MIN = height_control.get_float_param("ClimbAssist", "VzMin");

    // --- Target altitude (optional configurable) ---
    target_alt = height_control.get_float_param("Target", "Altitude");

    init = true;
}

/********************************************************************************
 * Test flight definition
 ********************************************************************************/
void test_flight(void)
{
    if (!init)
        init_flight();

    const float target_alt = 8.0f;
    float current_alt = g_mav_veh_rngfdr_current_distance * 0.01f;
    float alt_error = target_alt - current_alt;

    // --- PID output ---
    float pid_out = pid_thrust.pid3(
        Kp_thrust, Ki_thrust, Kd_thrust,
        alt_error, 0.0f, 0.0f,
        w1_thrust, 0.0f, 0.0f,
        ControlDim::Z, dt);

    // --- Adaptive climb assist ---
    static float climb_boost = 0.0f;

    const float CLIMB_BOOST_STEP = 0.005f;
    const float CLIMB_BOOST_MAX = 0.15f;
    const float CLIMB_BOOST_DECAY = 0.01f;
    const float ALT_TOL = 0.3f;   // within this, we’re "on target"
    const float SLOW_ZONE = 1.0f; // start slowing ascent 1m below target
    const float VZ_MIN = 0.15f;   // m/s threshold for “actually climbing”

    // Determine soft-slowing scale (0→far below, 1→within 1m)
    float approach_scale = 1.0f;
    if (alt_error < SLOW_ZONE && alt_error > 0.0f)
        approach_scale = alt_error / SLOW_ZONE;

    // Adaptive boost: increase only if below target and not rising fast
    if (alt_error > ALT_TOL && g_mav_veh_local_ned_vz < VZ_MIN)
    {
        // Scale boost growth by how close we are (less boost near target)
        climb_boost = std::min(climb_boost + CLIMB_BOOST_STEP * approach_scale, CLIMB_BOOST_MAX);
    }
    else if (alt_error < -ALT_TOL)
    {
        // Overshoot → decay faster
        climb_boost = std::max(climb_boost - 2.0f * CLIMB_BOOST_DECAY, 0.0f);
    }
    else
    {
        // Near target → slow decay
        climb_boost = std::max(climb_boost - CLIMB_BOOST_DECAY, 0.0f);
    }

    // Combine PID + boost
    float thrust_cmd = pid_out + climb_boost;

    // Within tolerance (±0.3 m) → blend slowly to hover thrust (to stop bouncing)
    if (std::fabs(alt_error) < ALT_TOL)
        thrust_cmd = 0.95f * thrust_cmd + 0.05f * hover_thrust;

    // Clamp safely
    thrust_cmd = std::clamp(thrust_cmd, 0.0f, 1.0f);

    // Apply thrust
    takeoff(thrust_cmd);

    spdlog::info("[ALT_CTRL] Alt: {:.2f} m | Err: {:.2f} | Vz: {:.2f} | PID: {:.3f} | Boost: {:.3f} | Cmd: {:.3f}",
                 current_alt, alt_error, g_mav_veh_local_ned_vz, pid_out, climb_boost, thrust_cmd);

    if (g_param_read)
    {
        hover_thrust = g_param_value;
        if (hover_thrust > 1.0f)
            hover_thrust = 0.36f;
        spdlog::info("[MAVLINK] MOT_THST_HOVER: {:.3f}", hover_thrust);
    }

    countupTimer();
}
