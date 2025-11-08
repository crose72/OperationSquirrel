/********************************************************************************
 * @file    path_planner.cpp
 * @author  Cameron Rose
 * @date    3/12/2025
 * @brief   The path planner contains the MPC setup and solving for the optimal
 *          flight path to deliver a payload to a moving target.  An external
 *          MPC library will be used to solve the MPC problem statement.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "path_planner.h"
#include "video_io.h"
#include "param_reader.h"
#include "pid.h"
#include "localize_target.h"
#include "time_calc.h"
#include "track_target.h"
#include "mav_data_hub.h"
#include "path_planner.h"
#include "signal_processing.h"
#include <spdlog/spdlog.h>

#include <algorithm> // for std::min/std::max
#include <cmath>     // for sin/cos/atan2

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

static inline float clampf(float v, float lo, float hi) { return std::max(lo, std::min(v, hi)); }

struct Vector2f
{
    float x = 0.f;
    float y = 0.f;
};

/* 1D jerk+accel limiter: tracks v toward v_cmd with bounded jerk & accel.
   Use one of these structs per dimension being controlled. */
struct MotionProfile1D
{
    float v = 0.f;      // shaped value
    float a = 0.f;      // internal accel state
    float a_max = 3.5f; // accel cap (m/s^2 or rad/s^2)
    float j_max = 8.0f; // jerk  cap (m/s^3 or rad/s^3)

    void update(float v_cmd, float dt, float k_v2a = 2.0f)
    {
        // desired accel is difference between the raw velocity
        // command and the last shaped velocity command
        const float a_des = (v_cmd - v) * k_v2a;

        // limit the acceleration to min/max jerk
        const float da = clampf(a_des - a, -j_max * dt, j_max * dt);

        // limit accel to min/max allowed accel
        a = clampf(a + da, -a_max, a_max);

        // update velocity command
        v += a * dt;
    }
};

/* 2D velocity shaper: independent 1D limiters for x & y */
struct MotionProfiler2D
{
    MotionProfile1D x;
    MotionProfile1D y;

    void setLimits(float a_max_xy, float j_max_xy)
    {
        x.a_max = y.a_max = a_max_xy;
        x.j_max = y.j_max = j_max_xy;
    }

    void reset(float vx = 0.f, float vy = 0.f)
    {
        x.v = vx;
        y.v = vy;
        x.a = y.a = 0.f;
    }

    void update(const Vector2f &v_cmd, float dt)
    {
        x.update(v_cmd.x, dt);
        y.update(v_cmd.y, dt);
    }

    Vector2f value() const { return {x.v, y.v}; }
};

/********************************************************************************
 * Object definitions
 ********************************************************************************/
PID pid_forwd;
PID pid_rev;
PID pid_yaw;
MotionProfiler2D g_vel_shaper; // shapes (g_vx_adjust, g_vy_adjust)
bool g_target_too_close;
bool yaw_initial_latched;
float g_x_error;
float g_y_error;
float g_vx_adjust;
float g_vy_adjust;
float g_vz_adjust;
float g_yaw_target;
float yaw_initial;
float min_yaw;
float max_yaw;
float g_yaw_adjust;
float g_mav_veh_yaw_prv;
float g_yaw_target_error;
float g_mav_veh_yaw_adjusted_for_playback;
float x_error_prv;
float vx_adjust_prv;
float vx_decel_prof_idx;
bool decel_profile_active;
int profile_sign;
bool target_valid_last_cycle;
float g_veh_vx_est;
float g_veh_vy_est;
float g_x_error_dot;
float x_error_dot_prv;
float g_vx_cmd_ff;
float g_vx_dot_des;
float g_target_sim_r; // predicted relative distance
float g_veh_sim_vd;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/
/* x forward */
float Kp_x = (float)0.5;
float Ki_x = (float)0.009;
float Kd_x = (float)0.1;
float w1_x = (float)1.0;
float w2_x = (float)0.0;
float w3_x = (float)0.0;

/* y forward */
float Kp_y = (float)0.03;
float Ki_y = (float)0.00;
float Kd_y = (float)0.001;
float w1_y = (float)1.0;
float w2_y = (float)0.0;
float w3_y = (float)0.0;

/* z forward */
float w1_z = (float)0.0;
float w2_z = (float)0.0;
float w3_z = (float)0.0;

/* x reverse */
float Kp_x_rev = (float)0.0;
float Ki_x_rev = (float)0.0;
float Kd_x_rev = (float)0.0;
float w1_x_rev = (float)1.0;
float w2_x_rev = (float)0.0;
float w3_x_rev = (float)0.0;

/* y reverse */
float Kp_y_rev = (float)0.005;
float Ki_y_rev = (float)0.0;
float Kd_y_rev = (float)0.0;
float w1_y_rev = (float)1.0;
float w2_y_rev = (float)0.0;
float w3_y_rev = (float)0.0;

float Kp_yaw = (float)0.08;
float Ki_yaw = (float)0.0;
float Kd_yaw = (float)0.00005;
float w1_yaw = (float)1.0;
float w2_yaw = (float)0.0;
float w3_yaw = (float)0.0;

float x_desired = (float)4.0;
float y_desired = (float)0.0;

float vxy_cmd_max_allowed_accel = (float)2.5; // m/s²  -> used as shaping accel cap
float vxy_cmd_max_allowed_jerk = (float)8.0;  // m/s^3 for x and y (jerk cap)
float x_error_dot_filt_coef = (float)0.0;
float vxy_cmd_ff_brake_gain = (float)0.0;
float alpha_ff_x = (float)0.0;
float beta_ff_x = (float)0.0;
float ff_accel_thresh = (float)0.2;
float ff_w_steady_state = (float)0.05;
float ff_w_transient = (float)0.1;

/********************************************************************************
 * Function definitions
 ********************************************************************************/
void get_path_params(void);
void calc_follow_error(void);
void calc_yaw_target_error(void);
void dtrmn_vel_cmd(void);
void calc_veh_speed(void);

/********************************************************************************
 * Function: get_path_params
 * Description: Read follow control parameters from a json or other file type.
 ********************************************************************************/
void get_path_params(void)
{
    ParamReader follow_control("../params.json");

    // Accessing Vel_PID_x parameters
    Kp_x = follow_control.get_float_param("PID_vx_forward", "Kp");
    Ki_x = follow_control.get_float_param("PID_vx_forward", "Ki");
    Kd_x = follow_control.get_float_param("PID_vx_forward", "Kd");
    w1_x = follow_control.get_float_param("PID_vx_forward", "w1");
    w2_x = follow_control.get_float_param("PID_vx_forward", "w2");
    w3_x = follow_control.get_float_param("PID_vx_forward", "w3");

    // Accessing Vel_PID_y parameters
    Kp_y = follow_control.get_float_param("PID_vy_forward", "Kp");
    Ki_y = follow_control.get_float_param("PID_vy_forward", "Ki");
    Kd_y = follow_control.get_float_param("PID_vy_forward", "Kd");
    w1_y = follow_control.get_float_param("PID_vy_forward", "w1");
    w2_y = follow_control.get_float_param("PID_vy_forward", "w2");
    w3_y = follow_control.get_float_param("PID_vy_forward", "w3");

    // Accessing Vel_PID_x parameters for reverse movement
    Kp_x_rev = follow_control.get_float_param("PID_vx_reverse", "Kp");
    Ki_x_rev = follow_control.get_float_param("PID_vx_reverse", "Ki");
    Kd_x_rev = follow_control.get_float_param("PID_vx_reverse", "Kd");
    w1_x_rev = follow_control.get_float_param("PID_vx_reverse", "w1");
    w2_x_rev = follow_control.get_float_param("PID_vx_reverse", "w2");
    w3_x_rev = follow_control.get_float_param("PID_vx_reverse", "w3");

    // Accessing Vel_PID_y parameters for reverse movment
    Kp_y_rev = follow_control.get_float_param("PID_vy_reverse", "Kp");
    Ki_y_rev = follow_control.get_float_param("PID_vy_reverse", "Ki");
    Kd_y_rev = follow_control.get_float_param("PID_vy_reverse", "Kd");
    w1_y_rev = follow_control.get_float_param("PID_vy_reverse", "w1");
    w2_y_rev = follow_control.get_float_param("PID_vy_reverse", "w2");
    w3_y_rev = follow_control.get_float_param("PID_vy_reverse", "w3");

    // Accessing Yaw PID parameters
    Kp_yaw = follow_control.get_float_param("PID_yaw", "Kp");
    Ki_yaw = follow_control.get_float_param("PID_yaw", "Ki");
    Kd_yaw = follow_control.get_float_param("PID_yaw", "Kd");
    w1_yaw = follow_control.get_float_param("PID_yaw", "w1");
    w2_yaw = follow_control.get_float_param("PID_yaw", "w2");
    w3_yaw = follow_control.get_float_param("PID_yaw", "w3");

    // Follow params
    x_desired = follow_control.get_float_param("Follow_Params", "Desired_X_offset");
    y_desired = follow_control.get_float_param("Follow_Params", "Desired_Y_offset");

    vxy_cmd_max_allowed_accel = follow_control.get_float_param("Follow_Params", "CMD_vxy_max_allowed_accel");
    vxy_cmd_max_allowed_jerk = follow_control.get_float_param("Follow_Params", "CMD_vxy_max_allowed_jerk");
    x_error_dot_filt_coef = follow_control.get_float_param("Follow_Params", "X_error_dot_filt");
    vxy_cmd_ff_brake_gain = follow_control.get_float_param("Follow_Params", "CMD_vxy_ff_brake_gain");
    alpha_ff_x = follow_control.get_float_param("Follow_Params", "FF_Alpha_X");
    beta_ff_x = follow_control.get_float_param("Follow_Params", "FF_Beta_X");
    ff_w_transient = follow_control.get_float_param("Follow_Params", "FF_weight_transient");
    ff_accel_thresh = follow_control.get_float_param("Follow_Params", "FF_accel_thresh");
    ff_w_steady_state = follow_control.get_float_param("Follow_Params", "FF_weight_steady_state");
}

/********************************************************************************
 * Function: calc_follow_error
 * Description: Calculate the error between the desired target offset and the
 *              target's actual offset.
 ********************************************************************************/
void calc_follow_error(void)
{
    if (g_x_target_ekf < 0.001f)
    {
        g_x_error = (float)0.0;
    }
    else
    {
        g_x_error = (g_x_target_ekf - x_desired);
    }

    // Currently set to 0 - not sending a y component velocity vector
    g_y_error = y_desired;

    // Error derivative
    if (!target_valid_last_cycle && g_target_valid ||
        !g_target_valid && target_valid_last_cycle)
    {
        g_x_error_dot = (float)0.0;
    }
    else
    {

        g_x_error_dot = (g_x_error - x_error_prv) / g_dt;
        g_x_error_dot = low_pass_filter(g_x_error_dot, x_error_dot_prv, x_error_dot_filt_coef);
    }

    x_error_dot_prv = g_x_error_dot;
}

void calc_veh_speed(void)
{
    g_veh_vx_est = cosf(g_mav_veh_yaw) * g_mav_veh_local_ned_vx + sinf(g_mav_veh_yaw) * g_mav_veh_local_ned_vy;
    g_veh_vy_est = cosf(g_mav_veh_yaw) * g_mav_veh_local_ned_vy - sinf(g_mav_veh_yaw) * g_mav_veh_local_ned_vx;
}

/********************************************************************************
 * Function: calc_yaw_target_error
 * Description: Calculate the yaw target for the drone based on the target
 *              position.
 ********************************************************************************/
void calc_yaw_target_error(void)
{
    // is g_use_video_playback and yaw_initial_latched needed here?
    // TODO: or is it fine for using camera to with drone turning all the time?
    if (!g_first_loop_after_start && g_use_video_playback && !yaw_initial_latched)
    {
        yaw_initial = g_mav_veh_yaw;
        yaw_initial_latched = true;
        // Yaw has a max value of M_PI and a min value of - M_PI.
        // If the sum of the yaw target and the initial latched yaw state of the
        // vehicle results in a value < -M_PI or > M_PI then the yaw value will have
        // to account for that, since when yaw becomes > M_PI or < -M_PI it rolls over
        // positive sign to negative or visa versa.
        float abs_min_yaw_temp = yaw_initial - g_cam0_fov_rad;

        if (abs_min_yaw_temp >= -M_PI)
        {
            min_yaw = abs_min_yaw_temp;
        }
        else
        {
            min_yaw = M_PI - (std::abs(abs_min_yaw_temp) - M_PI);
        }

        float abs_max_yaw_temp = yaw_initial + g_cam0_fov_rad;

        if (abs_max_yaw_temp <= M_PI)
        {
            max_yaw = abs_max_yaw_temp;
        }
        else
        {
            max_yaw = -(M_PI - (abs_max_yaw_temp - M_PI));
        }
    }

    g_mav_veh_yaw_adjusted_for_playback = g_mav_veh_yaw - yaw_initial;

    // If the target is to the right of the center of the video then yaw right
    // If the target is to the left of the center of the video then yaw left
    if (g_target_cntr_offset_y > 50.0)
    {
        g_yaw_target = (0.0011 * g_target_cntr_offset_y);
    }
    else if (g_target_cntr_offset_y < -50.0)
    {
        g_yaw_target = -(0.0011 * std::abs(g_target_cntr_offset_y));
    }
    else
    {
        g_yaw_target = 0.0;
    }

    const float angle = 2 * std::tan(g_camera_fov * 0.5) / g_cam0_video_width;

    const float deadband_px = (float)50.0;

    // Yaw in radians using pinhole model
    if (std::abs(g_target_cntr_offset_y) > deadband_px)
    {
        g_yaw_target = g_target_cntr_offset_y * angle;
    }
    else
    {
        g_yaw_target = 0.0;
    }

    // Corrections for when yaw changes signs
    float abs_yaw_target = 0.0;

    if ((g_yaw_target + g_mav_veh_yaw) < -M_PI)
    {
        abs_yaw_target = M_PI - (std::abs(g_yaw_target) - M_PI);
    }
    else if ((g_yaw_target + g_mav_veh_yaw) > M_PI)
    {
        abs_yaw_target = -(M_PI - (std::abs(g_yaw_target) - M_PI));
    }

    float abs_max_yaw_temp = yaw_initial + g_cam0_fov_rad;

    if (abs_max_yaw_temp <= M_PI)
    {
        max_yaw = abs_max_yaw_temp;
    }
    else
    {
        max_yaw = -(M_PI - (abs_max_yaw_temp - M_PI));
    }

    // If using video playback then subtract the initial yaw to track
    // the yaw error over time.  Otherwise, the error is just the yaw target
    // because the goal is for the target to be in the center of the frame.
    if (g_use_video_playback)
    {
        g_yaw_target_error = g_yaw_target - g_mav_veh_yaw_adjusted_for_playback;
    }
    else
    {
        g_yaw_target_error = g_yaw_target;
    }
}

/********************************************************************************
 * Function: dtrmn_vel_cmd
 * Description: Determine the follow vector based on the vehicle's error between
 *              the desired target offset and the actual target offset.
 ********************************************************************************/
void dtrmn_vel_cmd(void)
{
    g_target_too_close = (g_x_error < 0.0f);

    // PID control outputs - these are the control setpoints (vx, vy, etc)
    if (g_target_valid && g_target_too_close)
    {
        g_vx_adjust = pid_rev.pid3(Kp_x_rev, Ki_x_rev, Kd_x_rev,
                                   g_x_error, 0.0f, 0.0f,
                                   w1_x_rev, 0.0f, 0.0f, ControlDim::X, g_dt);

        g_vy_adjust = pid_rev.pid3(Kp_y_rev, Ki_y_rev, Kd_y_rev,
                                   g_y_error, 0.0f, 0.0f,
                                   w1_y_rev, 0.0f, 0.0f, ControlDim::Y, g_dt);

        g_yaw_adjust = pid_yaw.pid3(Kp_yaw, Ki_yaw, Kd_yaw,
                                    g_yaw_target_error, 0.0f, 0.0f,
                                    w1_yaw, 0.0f, 0.0f, ControlDim::YAW, g_dt);
    }
    else if (g_target_valid && !g_target_too_close)
    {
        g_vx_adjust = pid_forwd.pid3(Kp_x, Ki_x, Kd_x,
                                     g_x_error, 0.0f, 0.0f,
                                     w1_x, 0.0f, 0.0f, ControlDim::X, g_dt);

        g_vy_adjust = pid_forwd.pid3(Kp_y, Ki_y, Kd_y,
                                     g_y_error, 0.0f, 0.0f,
                                     w1_y, 0.0f, 0.0f, ControlDim::Y, g_dt);

        g_yaw_adjust = pid_yaw.pid3(Kp_yaw, Ki_yaw, Kd_yaw,
                                    g_yaw_target_error, 0.0f, 0.0f,
                                    w1_yaw, 0.0f, 0.0f, ControlDim::YAW, g_dt);
    }
    else
    {
        // Raw commands go to zero when target invalid
        g_vx_adjust = 0.0f;
        g_vy_adjust = 0.0f;
        g_yaw_adjust = 0.0f;
    }

    // PID provides the raw command to reach the target
    // Map your max-ramp-rate knob to the accel cap; jerk is separate.
    // This ensures any update (including update-to-zero) is turned into a smooth S-curve.
    // It does so by limiting the velocity command
    g_vel_shaper.update(Vector2f{g_vx_adjust, g_vy_adjust}, g_dt);
    const Vector2f v_shaped = g_vel_shaper.value();

    // Updated control setpoints with smoothing
    g_vx_adjust = v_shaped.x;
    g_vy_adjust = v_shaped.y;

    // -----------------------------------------------------------------------------
    // Feed-forward acceleration and proportional braking (X-direction only)
    // -----------------------------------------------------------------------------
    const float v_des = g_vx_adjust;        // PID output (desired velocity)
    const float v_des_prev = vx_adjust_prv; // previous desired velocity
    const float v_act = g_veh_vx_est;       // measured/estimated velocity
    const float dt = g_dt;

    // Desired acceleration (finite difference)
    const float vdot_des = (v_des - v_des_prev) / dt;

    // Feed-forward input to required to achieve desired velocity
    const float u_ff = (v_des + vdot_des * dt - alpha_ff_x * v_act) / beta_ff_x;

    // Determine if braking should be applied
    const bool braking_active = ((v_des * v_act > 0.0f) && (fabsf(v_des) < fabsf(v_act))) || (v_des * v_act < 0.0f);

    // Select the FF weighting factor based on the current vehicle behavior
    // If speeding up or braking then use more FF
    const float ff_w = (fabsf(vdot_des) > ff_accel_thresh || braking_active) ? ff_w_transient : ff_w_steady_state;

    // Blend PID setpoint with FF term
    float vx_cmd = (1.0f - ff_w) * v_des + ff_w * u_ff;

    // Apply proportional braking when slowing down or reversing
    if (braking_active)
    {
        vx_cmd -= vxy_cmd_ff_brake_gain * v_act;
    }

    // Output
    g_vx_cmd_ff = vx_cmd;
    g_vx_dot_des = vdot_des;

    target_valid_last_cycle = g_target_valid;
    vx_adjust_prv = v_des;
    x_error_prv = g_x_error;
}

struct FollowSimState
{
    float r;          // relative distance (target - drone)
    float v_d;        // drone forward velocity
    float v_des_prev; // previous desired velocity
};

void sim_follow_relative_step(
    FollowSimState &state,
    float dt,
    float alpha_plant,
    float beta_plant,
    float u_cmd, // actual control input (PID + FF)
    float v_rel, // EKF relative velocity
    float brake_gain)
{
    // --- Vehicle plant (for simulated drone velocity) ---
    float v_d = state.v_d;
    float v_d_next = alpha_plant * v_d + beta_plant * u_cmd;

    // --- Optional braking clamp ---
    if (((u_cmd * v_d) < 0.0f) && fabsf(v_d) > 0.1f)
        v_d_next -= brake_gain * v_d * dt;

    // --- Relative distance integration ---
    static float vrel_prev = 0.0f;
    float dv_rel = (v_rel - vrel_prev) / std::max(1e-3f, dt);
    vrel_prev = v_rel;

    const float T_lead = 0.20f; // 200 ms lead compensator
    const float k_corr = 0.15f;
    float gamma = 0.02f;

    // 🔧 Stronger correction when near target (error small)
    if (fabsf(g_x_error) < 2.0f)
        gamma = 0.06f;

    float v_rel_lead = v_rel + T_lead * dv_rel + k_corr * (g_x_error - state.r);

    // integrate relative distance
    float r_pred = state.r + v_rel_lead * dt;

    // complementary blend toward measured x_error
    float r_next = (1.0f - gamma) * r_pred + gamma * g_x_error;

    // --- Store back ---
    state.v_d = v_d_next;
    state.r = r_next;
}

void simulation(void)
{
    static FollowSimState sim_state{.r = 0.0f, .v_d = 0.0f, .v_des_prev = 0.0f};
    static bool sim_initialized = false;

    if (!g_target_valid)
        return;

    if (!sim_initialized)
    {
        sim_state.r = g_x_error;
        sim_state.v_d = g_veh_vx_est;
        sim_state.v_des_prev = 0.0f;
        sim_initialized = true;
    }

    const float dt = g_dt;

    // --- plant tuning ---
    const float alpha_plant = 0.94f;
    const float beta_plant = 0.04f; // 🔧 was 0.06f — reduces gain toward real dynamics

    // --- EKF relative velocity ---
    float v_rel = g_vx_target_ekf;

    // --- Simulate one step ---
    sim_follow_relative_step(
        sim_state,
        dt,
        alpha_plant,
        beta_plant,
        g_vx_adjust,
        v_rel,
        vxy_cmd_ff_brake_gain);

    // --- Outputs ---
    g_target_sim_r = sim_state.r;
    g_veh_sim_vd = sim_state.v_d;
}

/********************************************************************************
 * Function: PathPlanner
 * Description: PathPlanner class constructor.
 ********************************************************************************/
PathPlanner::PathPlanner(void) {}

/********************************************************************************
 * Function: ~PathPlanner
 * Description: PathPlanner class destructor.
 ********************************************************************************/
PathPlanner::~PathPlanner(void) {}

/********************************************************************************
 * Function: init
 * Description: Initialize all planner variables.  Run once at the start
 *              of the program.
 ********************************************************************************/
bool PathPlanner::init(void)
{
    get_path_params();

    g_target_too_close = false;
    g_x_error = (float)0.0;
    g_y_error = (float)0.0;
    g_vx_adjust = (float)0.0;
    g_vy_adjust = (float)0.0;
    g_vz_adjust = (float)0.0;
    g_yaw_target = (float)0.0;
    yaw_initial = (float)0.0;
    yaw_initial_latched = false;
    max_yaw = (float)0.0;
    min_yaw = (float)0.0;
    g_yaw_adjust = (float)0.0;
    g_mav_veh_yaw_prv = (float)0.0;
    g_yaw_target_error = (float)0.0;
    g_mav_veh_yaw_adjusted_for_playback = (float)0.0;
    x_error_prv = (float)0.0;
    vx_adjust_prv = (float)0.0;
    target_valid_last_cycle = false;
    g_vel_shaper.setLimits(/*a_max_xy=*/vxy_cmd_max_allowed_accel,
                           /*j_max_xy=*/vxy_cmd_max_allowed_jerk);
    g_vel_shaper.reset(/*vx=*/(float)0.0,
                       /*vy=*/(float)0.0);

    return true;
}

/********************************************************************************
 * Function: loop
 * Description: Function to run every loop.
 ********************************************************************************/
void PathPlanner::loop(void)
{
    calc_veh_speed();
    calc_follow_error();
    calc_yaw_target_error();
    dtrmn_vel_cmd();
    simulation();
}

/********************************************************************************
 * Function: shutdown
 * Description: Function to clean up planner at the end of the program.
 ********************************************************************************/
void PathPlanner::shutdown(void)
{
    // place clean up code here
}
