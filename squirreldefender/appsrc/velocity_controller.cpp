/********************************************************************************
 * @file    velocity_controller.cpp
 * @author  Cameron Rose
 * @date    3/12/2025
 * @brief   Computes velocity commands (vx, vy, yaw rate) for target following.
 *          Uses PID controllers, motion profiling (S-curve), and target error
 *          from the perception system to generate smoothed velocity inputs.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include <algorithm> // for std::min/std::max
#include <cmath>     // for sin/cos/atan2

#include <spdlog/spdlog.h>

#include "common_inc.h"
#include "velocity_controller.h"
#include "video_io.h"
#include "param_reader.h"
#include "pid.h"
#include "target_localization.h"
#include "time_calc.h"
#include "target_tracking.h"
#include "mav_data_hub.h"
#include "signal_processing.h"

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

// PID controllers
PID pid_forwd;
PID pid_rev;

// Motion profiling (velocity shaping / S-curve filtering)
MotionProfiler2D g_vel_shaper;

// Target tracking + position error state
bool g_tgt_too_close;
float g_pos_err_x;
float g_pos_err_y;
float g_pos_err_x_dot;
float x_error_prv;
float x_error_dot_prv;
bool target_valid_last_cycle;

// Vehicle velocity (body-frame estimates)
float g_veh_vel_x_est;
float g_veh_vel_y_est;

// Control command outputs (post-PID + shaping)
float g_ctrl_vel_x_cmd;
float g_ctrl_vel_y_cmd;
float g_ctrl_vel_z_cmd;
float g_ctrl_yaw_cmd;
float g_ctrl_yaw_tgt;

// Yaw handling and playback compensation
bool yaw_initial_latched;
float yaw_initial;
float g_veh_yaw_playback_adj;
float g_mav_veh_yaw_prv;
float g_yaw_err;
float min_yaw;
float max_yaw;
float yaw_rad_per_pix;

// Feed-forward braking / profile state
float vx_adjust_prv;
float vx_decel_prof_idx;
bool decel_profile_active;
int profile_sign;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/
/* x forward */
float kp_x = (float)0.5;
float ki_x = (float)0.009;
float kd_x = (float)0.1;

/* y forward */
float kp_y = (float)0.03;
float ki_y = (float)0.00;
float kd_y = (float)0.001;

/* z forward */

/* x reverse */
float kp_x_rev = (float)0.0;
float ki_x_rev = (float)0.0;
float kd_x_rev = (float)0.0;

/* y reverse */
float kp_y_rev = (float)0.005;
float ki_y_rev = (float)0.0;
float kd_y_rev = (float)0.0;

// yaw control
float kp_yaw = (float)0.08;
float ki_yaw = (float)0.0;
float kd_yaw = (float)0.00005;
float yaw_deadband_thresh = (float)50.0;

float des_follow_dist_x_m = (float)4.0;
float des_follow_dist_y_m = (float)0.0;

float vxy_cmd_max_allowed_accel = (float)2.5; // m/s²  -> used as shaping accel cap
float vxy_cmd_max_allowed_jerk = (float)8.0;  // m/s^3 for x and y (jerk cap)
float x_error_dot_filt_coef = (float)0.0;
float vxy_cmd_ff_brake_gain = (float)0.0;

/********************************************************************************
 * Function definitions
 ********************************************************************************/
void get_vel_ctrl_params(void);
void calc_veh_pos_err(void);
void calc_yaw_target_error(void);
void dtrmn_vel_cmd(void);
void calc_veh_speed(void);

/********************************************************************************
 * Function: get_vel_ctrl_params
 * Description: Set velocity control parameters.
 ********************************************************************************/
void get_vel_ctrl_params(void)
{
    ParamReader follow_control("../params.json");

    // Accessing Vel_PID_x parameters
    kp_x = follow_control.get_float_param("velocity_control.pid_vx_forward.kp");
    ki_x = follow_control.get_float_param("velocity_control.pid_vx_forward.ki");
    kd_x = follow_control.get_float_param("velocity_control.pid_vx_forward.kd");

    // Accessing Vel_PID_y parameters
    kp_y = follow_control.get_float_param("velocity_control.pid_vy_forward.kp");
    ki_y = follow_control.get_float_param("velocity_control.pid_vy_forward.ki");
    kd_y = follow_control.get_float_param("velocity_control.pid_vy_forward.kd");

    // Accessing Vel_PID_x parameters for reverse movement
    kp_x_rev = follow_control.get_float_param("velocity_control.pid_vx_reverse.kp");
    ki_x_rev = follow_control.get_float_param("velocity_control.pid_vx_reverse.ki");
    kd_x_rev = follow_control.get_float_param("velocity_control.pid_vx_reverse.kd");

    // Accessing Vel_PID_y parameters for reverse movment
    kp_y_rev = follow_control.get_float_param("velocity_control.pid_vy_reverse.kp");
    ki_y_rev = follow_control.get_float_param("velocity_control.pid_vy_reverse.ki");
    kd_y_rev = follow_control.get_float_param("velocity_control.pid_vy_reverse.kd");

    // Accessing Yaw PID parameters
    kp_yaw = follow_control.get_float_param("velocity_control.pid_yaw_forward.kp");
    ki_yaw = follow_control.get_float_param("velocity_control.pid_yaw_forward.ki");
    kd_yaw = follow_control.get_float_param("velocity_control.pid_yaw_forward.kd");
    yaw_deadband_thresh = follow_control.get_float_param("velocity_control.yaw_deadband_thresh");

    // Follow params
    des_follow_dist_x_m = follow_control.get_float_param("velocity_control.des_follow_dist_x_m");
    des_follow_dist_y_m = follow_control.get_float_param("velocity_control.des_follow_dist_y_m");

    vxy_cmd_max_allowed_accel = follow_control.get_float_param("velocity_control.max_accel_mps2");
    vxy_cmd_max_allowed_jerk = follow_control.get_float_param("velocity_control.max_jerk_mps3");
    x_error_dot_filt_coef = follow_control.get_float_param("velocity_control.follow_dist_error_dot_filt_coef");
    vxy_cmd_ff_brake_gain = follow_control.get_float_param("velocity_control.velocity_ff_brake_gain");
}

/********************************************************************************
 * Function: calc_veh_pos_err
 * Description: Calculate the error between the desired target offset and the
 *              target's actual offset.
 ********************************************************************************/
void calc_veh_pos_err(void)
{
    if (g_tgt_pos_x_est < 0.001f)
    {
        g_pos_err_x = (float)0.0;
    }
    else
    {
        g_pos_err_x = (g_tgt_pos_x_est - des_follow_dist_x_m);
    }

    // Currently set to 0 - not sending a y component velocity vector
    g_pos_err_y = des_follow_dist_y_m;

    // Error derivative
    if ((!target_valid_last_cycle && g_tgt_valid) ||
        (!g_tgt_valid && target_valid_last_cycle))
    {
        g_pos_err_x_dot = (float)0.0;
    }
    else
    {

        g_pos_err_x_dot = div_zero_protect((g_pos_err_x - x_error_prv), g_app_dt);
        g_pos_err_x_dot = low_pass_filter(g_pos_err_x_dot, x_error_dot_prv, x_error_dot_filt_coef);
    }

    x_error_dot_prv = g_pos_err_x_dot;
}

void calc_veh_speed(void)
{
    g_veh_vel_x_est = cosf(g_mav_veh_yaw_rad) * g_mav_veh_vel_ned_x + sinf(g_mav_veh_yaw_rad) * g_mav_veh_vel_ned_y;
    g_veh_vel_y_est = cosf(g_mav_veh_yaw_rad) * g_mav_veh_vel_ned_y - sinf(g_mav_veh_yaw_rad) * g_mav_veh_vel_ned_x;
}

/********************************************************************************
 * Function: calc_yaw_target_error
 * Description: Calculate the yaw target for the drone based on the target
 *              position.
 ********************************************************************************/
void calc_yaw_target_error(void)
{
    // Latch the yaw angle where video playback started
    if (!g_app_first_loop && g_app_use_video_playback && !yaw_initial_latched)
    {
        yaw_initial = g_mav_veh_yaw_rad;
        yaw_initial_latched = true;
        // Yaw has a max value of M_PI and a min value of - M_PI.
        // If the sum of the yaw target and the initial latched yaw state of the
        // vehicle results in a value < -M_PI or > M_PI then the yaw value will have
        // to account for that, since when yaw becomes > M_PI or < -M_PI it rolls over
        // positive sign to negative or visa versa.
        float abs_min_yaw_raw = yaw_initial - g_cam0_fov_rad;

        if (abs_min_yaw_raw >= -M_PI)
        {
            min_yaw = abs_min_yaw_raw;
        }
        else
        {
            min_yaw = M_PI - (std::abs(abs_min_yaw_raw) - M_PI);
        }

        float abs_max_yaw_raw = yaw_initial + g_cam0_fov_rad;

        if (abs_max_yaw_raw <= M_PI)
        {
            max_yaw = abs_max_yaw_raw;
        }
        else
        {
            max_yaw = -(M_PI - (abs_max_yaw_raw - M_PI));
        }
    }

    g_veh_yaw_playback_adj = g_mav_veh_yaw_rad - yaw_initial;

    // If the target is to the right of the center of the video then yaw right
    // If the target is to the left of the center of the video then yaw left
    // Yaw in radians using pinhole model
    if (std::abs(g_tgt_cntr_offset_x_pix) > yaw_deadband_thresh)
    {
        g_ctrl_yaw_tgt = g_tgt_cntr_offset_x_pix * yaw_rad_per_pix;
    }
    else
    {
        g_ctrl_yaw_tgt = (float)0.0;
    }

    // If using video playback then subtract the initial yaw to track
    // the yaw error over time.  Otherwise, the error is just the yaw target
    // because the goal is for the target to be in the center of the frame.
    if (g_app_use_video_playback)
    {
        g_yaw_err = g_ctrl_yaw_tgt - g_veh_yaw_playback_adj;
    }
    else
    {
        g_yaw_err = g_ctrl_yaw_tgt;
    }
}

/********************************************************************************
 * Function: dtrmn_vel_cmd
 * Description: Determine the follow vector based on the vehicle's error between
 *              the desired target offset and the actual target offset.
 ********************************************************************************/
void dtrmn_vel_cmd(void)
{
    g_tgt_too_close = (g_pos_err_x < 0.0f);

    // PID control outputs - these are the control setpoints (vx, vy, etc)
    if (g_tgt_valid && g_tgt_too_close)
    {
        g_ctrl_vel_x_cmd = pid_rev.pid(kp_x_rev, ki_x_rev, kd_x_rev,
                                       g_pos_err_x, ControlDim::X, g_app_dt);

        g_ctrl_vel_y_cmd = pid_rev.pid(kp_y_rev, ki_y_rev, kd_y_rev,
                                       g_pos_err_y, ControlDim::Y, g_app_dt);

        g_ctrl_yaw_cmd = pid_rev.pid(kp_yaw, ki_yaw, kd_yaw,
                                     g_yaw_err, ControlDim::YAW, g_app_dt);
    }
    else if (g_tgt_valid && !g_tgt_too_close)
    {
        g_ctrl_vel_x_cmd = pid_forwd.pid(kp_x, ki_x, kd_x,
                                         g_pos_err_x, ControlDim::X, g_app_dt);

        g_ctrl_vel_y_cmd = pid_forwd.pid(kp_y, ki_y, kd_y,
                                         g_pos_err_y, ControlDim::Y, g_app_dt);

        g_ctrl_yaw_cmd = pid_forwd.pid(kp_yaw, ki_yaw, kd_yaw,
                                       g_yaw_err, ControlDim::YAW, g_app_dt);
    }
    else
    {
        // Raw commands go to zero when target invalid
        g_ctrl_vel_x_cmd = 0.0f;
        g_ctrl_vel_y_cmd = 0.0f;
        g_ctrl_yaw_cmd = 0.0f;
    }

    // PID provides the raw command to reach the target
    // Map your max-ramp-rate knob to the accel cap; jerk is separate.
    // This ensures any update (including update-to-zero) is turned into a smooth S-curve.
    // It does so by limiting the velocity command
    g_vel_shaper.update(Vector2f{g_ctrl_vel_x_cmd, g_ctrl_vel_y_cmd}, g_app_dt);
    const Vector2f v_shaped = g_vel_shaper.value();

    // Updated control setpoints with smoothing
    g_ctrl_vel_x_cmd = v_shaped.x;
    g_ctrl_vel_y_cmd = v_shaped.y;

    target_valid_last_cycle = g_tgt_valid;
    vx_adjust_prv = g_ctrl_vel_x_cmd;
    x_error_prv = g_pos_err_x;
}

void dtrmn_vel_ff_cmd(void)
{
    //
}

/********************************************************************************
 * Function: VelocityController
 * Description: Constructor
 ********************************************************************************/
VelocityController::VelocityController(void) {}

/********************************************************************************
 * Function: ~VelocityController
 * Description: Destructor
 ********************************************************************************/
VelocityController::~VelocityController(void) {}

/********************************************************************************
 * Function: init
 * Description: Initialize velocity controller.
 ********************************************************************************/
bool VelocityController::init(void)
{
    get_vel_ctrl_params();

    // TODO: FOV should be in radians - swap for radians version and then update PID for new units
    // pinhole camera model
    yaw_rad_per_pix = (float)2.0 * std::tan(g_cam0_fov_deg * (float)0.5) / g_cam0_img_width_px;

    g_vel_shaper.setLimits(/*a_max_xy=*/vxy_cmd_max_allowed_accel,
                           /*j_max_xy=*/vxy_cmd_max_allowed_jerk);
    g_vel_shaper.reset(/*vx=*/(float)0.0,
                       /*vy=*/(float)0.0);

    return true;
}

/********************************************************************************
 * Function: loop
 * Description: Run the main code.
 ********************************************************************************/
void VelocityController::loop(void)
{
    calc_veh_speed();
    calc_veh_pos_err();
    calc_yaw_target_error();
    dtrmn_vel_cmd();
    dtrmn_vel_ff_cmd();
}

/********************************************************************************
 * Function: shutdown
 * Description: Cleanup velocity controller.
 ********************************************************************************/
void VelocityController::shutdown(void)
{
    // place clean up code here
}
