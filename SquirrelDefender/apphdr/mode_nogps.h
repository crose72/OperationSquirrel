/********************************************************************************
 * @file    mode_nogps.h
 * @author  Cameron Rose
 * @date    10/29/2025
 * @brief   No-GPS guided flight mode controller.
 ********************************************************************************/
#include "common_inc.h"
#include "mav_utils.h"
#include "pid.h"

/********************************************************************************
 * @class ModeGuidedNoGPS
 * @brief Handles guided takeoff, altitude control, and basic vertical flight
 *        behavior when GPS is unavailable.
 ********************************************************************************/
class ModeGuidedNoGPS
{

public:
    enum TakeoffState
    {
        SPOOLUP,
        LIFT,
        ASCEND,
        DONE
    };

    ModeGuidedNoGPS();
    ArduPilotMode mode_number() const { return ArduPilotMode::GUIDED_NOGPS; };

    bool init();
    void run();
    float z_controller(void);

private:
    TakeoffState m_takeoff_state = SPOOLUP;
    PID m_pid;

    // --- altitude pid defaults ---
    float m_kp_alt = 0.6f;
    float m_ki_alt = 0.05f;
    float m_kd_alt = 0.15f;
    float m_w1_alt = 1.0f;
    float m_tau_alt = 0.15f;
    float m_decay_alt = 0.98f;
    float m_max_int_alt = 10.0f;

    // --- velocity pid defaults ---
    float m_kp_vel = 0.4f;
    float m_ki_vel = 0.1f;
    float m_kd_vel = 0.05f;
    float m_w1_vel = 1.0f;
    float m_tau_vel = 0.10f;
    float m_decay_vel = 0.98f;
    float m_max_int_vel = 10.0f;

    // --- takeoff parameters ---
    float m_takeoff_thr_max = 0.9f;
    float m_takeoff_thr_slew_time = 2.0f;
    float m_alt_tolerance = 0.3f;
    float m_climb_detect_vz = 0.1f;
    float m_climb_detect_alt = 0.3f;

    // --- other ---
    float m_hover_thrust = 0.35f;
    float m_takeoff_target_alt = 8.0f;

    // --- general controller parameters ---
    float m_dt = 0.05f;             // 20 Hz loop
    float m_vz_max = 2.5f;          // max climb/descend rate [m/s]
    float m_thr_delta_min = -0.20f; // min thrust delta
    float m_thr_delta_max = 0.50f;  // max thrust delta
    float m_min_alt_valid = 0.05f;  // avoid 0 division / noise
    bool m_initialized = false;
};