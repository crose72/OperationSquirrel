/********************************************************************************
 * @file    mode_nogps.cpp
 * @author  Cameron Rose
 * @date    10/29/2025
 * @brief   No-GPS guided mode implementation.
 ********************************************************************************/
#include "mode_nogps.h"
#include "mav_utils.h"
#include "mav_data_hub.h"
#include "pid.h"
#include "param_reader.h"
#include <spdlog/spdlog.h>

/********************************************************************************
 * Constructor
 ********************************************************************************/
ModeGuidedNoGPS::ModeGuidedNoGPS() {}

/********************************************************************************
 * init
 ********************************************************************************/
bool ModeGuidedNoGPS::init()
{
}

/********************************************************************************
 * update
 ********************************************************************************/
void ModeGuidedNoGPS::run(void)
{
}

/********************************************************************************
 * compute_takeoff_thrust
 ********************************************************************************/
float ModeGuidedNoGPS::z_controller()
{
    // --- sensor readings ---
    const float current_alt = std::max(g_mav_veh_rngfdr_current_distance * 0.01f, m_min_alt_valid);
    const float vz_up = -g_mav_veh_local_ned_vz; // +up in NED

    // --- outer loop: altitude pid -> climb rate target ---
    const float alt_error = m_takeoff_target_alt - current_alt;
    float vz_target = m_pid.pid(m_kp_alt, m_ki_alt, m_kd_alt, alt_error, ControlDim::Z, m_dt);
    vz_target = std::clamp(vz_target, -m_vz_max, m_vz_max);

    // --- inner loop: velocity pid -> thrust delta ---
    const float vz_error = vz_target - vz_up;
    float thrust_delta = m_pid.pid(m_kp_vel, m_ki_vel, m_kd_vel, vz_error, ControlDim::THRUST, m_dt);
    thrust_delta = std::clamp(thrust_delta, m_thr_delta_min, m_thr_delta_max);

    // --- combine ---
    float thrust_cmd = m_hover_thrust + thrust_delta;
    thrust_cmd = std::clamp(thrust_cmd, 0.0f, m_takeoff_thr_max);

    // --- completion detection ---
    if (std::fabs(alt_error) < m_alt_tolerance && std::fabs(vz_up) < 0.05f)
    {
        m_takeoff_state = TakeoffState::DONE;
    }

    spdlog::info("[nogps] alt:{:.2f} err:{:.2f} vz:{:.2f} vz_tgt:{:.2f} thr:{:.3f}",
                 current_alt, alt_error, vz_up, vz_target, thrust_cmd);

    return thrust_cmd;
}

/********************************************************************************
 * send_takeoff_command
 ********************************************************************************/
// void ModeGuidedNoGPS::send_takeoff_command(float thrust_cmd)
// {
//     float q[4] = {1, 0, 0, 0}; // level attitude
//     float thrust_body[3] = {0, 0, 0};

//     MavMotion::send_cmd_set_attitude_target(
//         SENDER_SYS_ID,
//         SENDER_COMP_ID,
//         TARGET_SYS_ID,
//         TARGET_COMP_ID,
//         0b00000111, // ignore body rates
//         q,
//         0, 0, 0,
//         thrust_cmd,
//         thrust_body);
// }
