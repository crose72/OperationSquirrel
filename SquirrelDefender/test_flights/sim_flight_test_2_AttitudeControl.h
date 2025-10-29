/********************************************************************************
 * @file    sim_flight_test_2_AttitudeControl.h
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   A test to understand the set attitude target message for controlling
 *          the drone.
 ********************************************************************************/

#include "common_inc.h"
#include "mav_data_hub.h"
#include "mav_utils.h"
#include <spdlog/spdlog.h>
#include "pid.h"
#include "param_reader.h"
#include "system_controller.h"

/********************************************************************************
 * Global and state definitions
 ********************************************************************************/
float timerVal = 0.0f;
int stage = 0;
bool init = false;
// --- PID controllers ---
PID pid_alt; // altitude -> velocity
PID pid_vel; // velocity -> thrust

PID pid_thrust;
float alt_error = 0.0f;

// --- PID Thrust Parameters ---
float Kp_thrust = 0.0f;
float Ki_thrust = 0.0f;
float Kd_thrust = 0.0f;
float w1_thrust = 0.0f;
float Kp_alt;
float Ki_alt;
float Kd_alt;
float w1_alt;
float Kp_vel;
float Ki_vel;
float Kd_vel;
float w1_vel;

// --- Hover & Target ---
float hover_thrust = 0.35f;      // default if param read fails
float takeoff_target_alt = 8.0f; // meters

// --- Takeoff Parameters ---
float TAKEOFF_THR_MAX = 0.9f;
float TAKEOFF_THR_SLEW_TIME = 2.0f; // seconds
float ALT_TOL = 0.3f;
float CLIMB_DETECT_VZ = 0.1f;  // m/s threshold for "moving up"
float CLIMB_DETECT_ALT = 0.3f; // m threshold for "already lifted"

// --- Takeoff state machine ---
enum TakeoffState
{
    TAKEOFF_SPOOLUP,
    TAKEOFF_LIFT,
    TAKEOFF_ASCEND,
    TAKEOFF_DONE
};
TakeoffState takeoff_state = TAKEOFF_SPOOLUP;

/********************************************************************************
 * Utility functions
 ********************************************************************************/
void countupTimer(void) { timerVal += 0.05f; }
void resetTimer(void) { timerVal = 0.0f; }

void takeoff(float thrust_value)
{
    // Send MAVLink attitude target with desired thrust
    float q[4] = {1, 0, 0, 0}; // flat attitude
    float thrust_body[3] = {0, 0, 0};

    MavMotion::send_cmd_set_attitude_target(
        SENDER_SYS_ID,
        SENDER_COMP_ID,
        TARGET_SYS_ID,
        TARGET_COMP_ID,
        0b00000111, // ignore rates, use attitude+thrust
        q,
        0, 0, 0,
        thrust_value,
        thrust_body);
}

bool motors_ready(void)
{
    return (g_system_state == SystemState::STANDBY);
}

/********************************************************************************
 * Init flight
 ********************************************************************************/
void init_flight(void)
{
    ParamReader height_control("../params.json");

    // --- PID Parameters ---
    Kp_thrust = height_control.get_float_param("PID_thrust", "Kp");
    Ki_thrust = height_control.get_float_param("PID_thrust", "Ki");
    Kd_thrust = height_control.get_float_param("PID_thrust", "Kd");
    w1_thrust = height_control.get_float_param("PID_thrust", "w1");

    Kp_alt = height_control.get_float_param("PID_alt", "Kp");
    Ki_alt = height_control.get_float_param("PID_alt", "Ki");
    Kd_alt = height_control.get_float_param("PID_alt", "Kd");
    w1_alt = height_control.get_float_param("PID_alt", "w1");

    Kp_vel = height_control.get_float_param("PID_vel", "Kp");
    Ki_vel = height_control.get_float_param("PID_vel", "Ki");
    Kd_vel = height_control.get_float_param("PID_vel", "Kd");
    w1_vel = height_control.get_float_param("PID_vel", "w1");

    // --- Takeoff Parameters ---
    TAKEOFF_THR_MAX = height_control.get_float_param("Takeoff", "ThrottleMax");
    TAKEOFF_THR_SLEW_TIME = height_control.get_float_param("Takeoff", "ThrottleSlewTime");
    ALT_TOL = height_control.get_float_param("Takeoff", "AltTolerance");
    CLIMB_DETECT_VZ = height_control.get_float_param("Takeoff", "ClimbDetectVz");
    CLIMB_DETECT_ALT = height_control.get_float_param("Takeoff", "ClimbDetectAlt");

    // --- Target Altitude ---
    takeoff_target_alt = height_control.get_float_param("Target", "Altitude");

    // --- MAVLink setup ---
    MavCmd::read_param(SENDER_SYS_ID, SENDER_COMP_ID, TARGET_SYS_ID, TARGET_COMP_ID,
                       "MOT_THST_HOVER", -1);
    MavCmd::set_mode_guided_nogps(SENDER_SYS_ID, SENDER_COMP_ID, TARGET_SYS_ID, TARGET_COMP_ID);
    MavCmd::arm_vehicle(SENDER_SYS_ID, SENDER_COMP_ID, TARGET_SYS_ID, TARGET_COMP_ID);

    takeoff_state = TAKEOFF_SPOOLUP;
    init = true;
}

/********************************************************************************
 * Test flight logic
 ********************************************************************************/
void test_flight(void)
{
    if (!init)
        init_flight();

    const float dt = 0.05f; // 20 Hz loop
    static float throttle = 0.0f;

    float current_alt = g_mav_veh_rngfdr_current_distance * 0.01f;
    float alt_error = takeoff_target_alt - current_alt;
    float vz_up = -g_mav_veh_local_ned_vz; // +up

    // --- Outer loop: altitude PID -> climb rate target ---
    float vz_target = pid_alt.pid3(
        Kp_alt, Ki_alt, Kd_alt,
        alt_error, 0, 0,
        w1_alt, 0, 0, ControlDim::Z, dt);

    // limit climb/descend rate (like ArduPilot)
    const float VZ_MAX = 1.0f; // m/s
    vz_target = std::clamp(vz_target, -VZ_MAX, VZ_MAX);

    // --- Inner loop: velocity PID -> thrust delta ---
    float vz_error = vz_target - vz_up;
    float thrust_delta = pid_vel.pid3(
        Kp_vel, Ki_vel, Kd_vel,
        vz_error, 0, 0,
        w1_vel, 0, 0, ControlDim::Z, dt);

    // clamp thrust delta to ±20% of hover
    thrust_delta = std::clamp(thrust_delta, -0.20f, 0.20f);

    // --- Combine ---
    float thrust_cmd = hover_thrust + thrust_delta;
    thrust_cmd = std::clamp(thrust_cmd, 0.0f, 1.0f);

    takeoff(thrust_cmd);

    // --- Completion check ---
    if (std::fabs(alt_error) < ALT_TOL && std::fabs(vz_up) < 0.05f)
        takeoff_state = TAKEOFF_DONE;

    spdlog::info("[CASCADE] alt:{:.2f} err:{:.2f} vz:{:.2f} vz_tgt:{:.2f} thr:{:.3f}",
                 current_alt, alt_error, vz_up, vz_target, thrust_cmd);
}
