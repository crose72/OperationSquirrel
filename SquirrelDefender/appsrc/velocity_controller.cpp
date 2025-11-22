/********************************************************************************
 * @file    velocity_controller.cpp
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
#include "velocity_controller.h"
#include "video_io.h"
#include "param_reader.h"
#include "pid.h"
#include "target_localization.h"
#include "time_calc.h"
#include "target_tracking.h"
#include "mav_data_hub.h"
#include "velocity_controller.h"
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
MotionProfiler2D g_vel_shaper; // shapes (g_ctrl_vel_x_cmd, g_ctrl_vel_y_cmd)
bool g_tgt_too_close;
bool yaw_initial_latched;
float g_pos_err_x;
float g_pos_err_y;
float g_ctrl_vel_x_cmd;
float g_ctrl_vel_y_cmd;
float g_ctrl_vel_z_cmd;
float g_ctrl_yaw_tgt;
float yaw_initial;
float min_yaw;
float max_yaw;
float g_ctrl_yaw_cmd;
float g_mav_veh_yaw_prv;
float g_yaw_err;
float g_veh_yaw_playback_adj;
float x_error_prv;
float vx_adjust_prv;
float vx_decel_prof_idx;
bool decel_profile_active;
int profile_sign;
bool target_valid_last_cycle;
float g_veh_vel_x_est;
float g_veh_vel_y_est;
float g_pos_err_x_dot;
float x_error_dot_prv;
float g_ctrl_prdtd_time_to_stop;
float g_ctrl_prdtd_time_to_reach_tgt;
bool g_ctrl_apprchng_tgt;
bool g_ctrl_brake_cmd;
float g_veh_acc_x_est;
float veh_vel_x_est_prv;
float g_sim_veh_vel_x_est;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/
/* x forward */
float Kp_x = (float)0.5;
float Ki_x = (float)0.009;
float Kd_x = (float)0.1;

/* y forward */
float Kp_y = (float)0.03;
float Ki_y = (float)0.00;
float Kd_y = (float)0.001;

/* z forward */

/* x reverse */
float Kp_x_rev = (float)0.0;
float Ki_x_rev = (float)0.0;
float Kd_x_rev = (float)0.0;

/* y reverse */
float Kp_y_rev = (float)0.005;
float Ki_y_rev = (float)0.0;
float Kd_y_rev = (float)0.0;

float Kp_yaw = (float)0.08;
float Ki_yaw = (float)0.0;
float Kd_yaw = (float)0.00005;

float x_desired = (float)4.0;
float y_desired = (float)0.0;

float vxy_cmd_max_allowed_accel = (float)2.5; // m/s²  -> used as shaping accel cap
float vxy_cmd_max_allowed_jerk = (float)8.0;  // m/s^3 for x and y (jerk cap)
float x_error_dot_filt_coef = (float)0.0;
float vxy_cmd_ff_brake_gain = (float)0.0;
float ctrl_arx_veh_vel_alpha = (float)0.0;
float ctrl_arx_veh_vel_beta = (float)0.0;
float ctrl_ebrake_vel_desired = (float)0.0;
float ctrl_ebrake_dist_thresh = (float)0.0;
float ctrl_ebrake_vel_thresh = (float)0.0;

/********************************************************************************
 * Function definitions
 ********************************************************************************/
void get_path_params(void);
void calc_follow_error(void);
void calc_yaw_target_error(void);
void dtrmn_vel_cmd(void);
void dtrmn_mod_vel_cmd(void);
void calc_veh_speed(void);

/********************************************************************************
 * Function: get_path_params
 * Description: Read follow control parameters from a json or other file type.
 ********************************************************************************/
void get_path_params(void)
{
    ParamReader vel_ctrl_params("../params.json");

    // Accessing Vel_PID_x parameters
    Kp_x = vel_ctrl_params.get_float_param("velocity_control.pid_vx_forward.kp");
    Ki_x = vel_ctrl_params.get_float_param("velocity_control.pid_vx_forward.ki");
    Kd_x = vel_ctrl_params.get_float_param("velocity_control.pid_vx_forward.kd");

    // Accessing Vel_PID_y parameters
    Kp_y = vel_ctrl_params.get_float_param("velocity_control.pid_vy_forward.kp");
    Ki_y = vel_ctrl_params.get_float_param("velocity_control.pid_vy_forward.ki");
    Kd_y = vel_ctrl_params.get_float_param("velocity_control.pid_vy_forward.kd");

    // Accessing Vel_PID_x parameters for reverse movement
    Kp_x_rev = vel_ctrl_params.get_float_param("velocity_control.pid_vx_reverse.kp");
    Ki_x_rev = vel_ctrl_params.get_float_param("velocity_control.pid_vx_reverse.ki");
    Kd_x_rev = vel_ctrl_params.get_float_param("velocity_control.pid_vx_reverse.kd");

    // Accessing Vel_PID_y parameters for reverse movment
    Kp_y_rev = vel_ctrl_params.get_float_param("velocity_control.pid_vy_reverse.kp");
    Ki_y_rev = vel_ctrl_params.get_float_param("velocity_control.pid_vy_reverse.ki");
    Kd_y_rev = vel_ctrl_params.get_float_param("velocity_control.pid_vy_reverse.kd");

    // Accessing Yaw PID parameters
    Kp_yaw = vel_ctrl_params.get_float_param("velocity_control.pid_yaw_forward.kp");
    Ki_yaw = vel_ctrl_params.get_float_param("velocity_control.pid_yaw_forward.ki");
    Kd_yaw = vel_ctrl_params.get_float_param("velocity_control.pid_yaw_forward.kd");

    // Follow params
    x_desired = vel_ctrl_params.get_float_param("velocity_control.follow_dist_x_m");
    y_desired = vel_ctrl_params.get_float_param("velocity_control.follow_dist_y_m");

    vxy_cmd_max_allowed_accel = vel_ctrl_params.get_float_param("velocity_control.max_accel_mps2");
    vxy_cmd_max_allowed_jerk = vel_ctrl_params.get_float_param("velocity_control.max_jerk_mps3");
    x_error_dot_filt_coef = vel_ctrl_params.get_float_param("velocity_control.follow_dist_error_dot_filt_coef");
    vxy_cmd_ff_brake_gain = vel_ctrl_params.get_float_param("velocity_control.velocity_ff_brake_gain");

    ctrl_arx_veh_vel_alpha = vel_ctrl_params.get_float_param("velocity_control.arx_veh_vel_model_alpha");
    ctrl_arx_veh_vel_beta = vel_ctrl_params.get_float_param("velocity_control.arx_veh_vel_model_beta");
    ctrl_ebrake_vel_desired = vel_ctrl_params.get_float_param("velocity_control.e_brake_vel_desired");
    ctrl_ebrake_dist_thresh = vel_ctrl_params.get_float_param("velocity_control.min_e_brake_dist_thresh");
    ctrl_ebrake_vel_thresh = vel_ctrl_params.get_float_param("velocity_control.min_e_brake_vel_thresh");
}

/********************************************************************************
 * Function: calc_follow_error
 * Description: Calculate the error between the desired target offset and the
 *              target's actual offset.
 ********************************************************************************/
void calc_follow_error(void)
{
    if (g_tgt_pos_x_est < 0.001f)
    {
        g_pos_err_x = (float)0.0;
    }
    else
    {
        g_pos_err_x = (g_tgt_pos_x_est - x_desired);
    }

    // Currently set to 0 - not sending a y component velocity vector
    g_pos_err_y = y_desired;

    // Error derivative
    if (!target_valid_last_cycle && g_tgt_valid ||
        !g_tgt_valid && target_valid_last_cycle)
    {
        g_pos_err_x_dot = (float)0.0;
    }
    else
    {

        g_pos_err_x_dot = div((g_pos_err_x - x_error_prv), g_app_dt);
        g_pos_err_x_dot = low_pass_filter(g_pos_err_x_dot, x_error_dot_prv, x_error_dot_filt_coef);
    }

    x_error_dot_prv = g_pos_err_x_dot;
}

void calc_veh_speed(void)
{
    g_veh_vel_x_est = cosf(g_mav_veh_yaw_rad) * g_mav_veh_vel_ned_x + sinf(g_mav_veh_yaw_rad) * g_mav_veh_vel_ned_y;
    g_veh_vel_y_est = cosf(g_mav_veh_yaw_rad) * g_mav_veh_vel_ned_y - sinf(g_mav_veh_yaw_rad) * g_mav_veh_vel_ned_x;
    g_veh_acc_x_est = div((g_veh_vel_x_est - veh_vel_x_est_prv), g_app_dt);
    veh_vel_x_est_prv = g_veh_vel_x_est;
}

/********************************************************************************
 * Function: calc_yaw_target_error
 * Description: Calculate the yaw target for the drone based on the target
 *              position.
 ********************************************************************************/
void calc_yaw_target_error(void)
{
    // is g_app_use_video_playback and yaw_initial_latched needed here?
    // TODO: or is it fine for using camera to with drone turning all the time?
    if (!g_app_first_loop && g_app_use_video_playback && !yaw_initial_latched)
    {
        yaw_initial = g_mav_veh_yaw_rad;
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

    g_veh_yaw_playback_adj = g_mav_veh_yaw_rad - yaw_initial;

    // If the target is to the right of the center of the video then yaw right
    // If the target is to the left of the center of the video then yaw left
    if (g_tgt_cntr_offset_x_pix > 50.0)
    {
        g_ctrl_yaw_tgt = (0.0011 * g_tgt_cntr_offset_x_pix);
    }
    else if (g_tgt_cntr_offset_x_pix < -50.0)
    {
        g_ctrl_yaw_tgt = -(0.0011 * std::abs(g_tgt_cntr_offset_x_pix));
    }
    else
    {
        g_ctrl_yaw_tgt = 0.0;
    }

    const float angle = 2 * std::tan(g_cam0_fov_deg * 0.5) / g_cam0_img_width_px;

    const float deadband_px = (float)50.0;

    // Yaw in radians using pinhole model
    if (std::abs(g_tgt_cntr_offset_x_pix) > deadband_px)
    {
        g_ctrl_yaw_tgt = g_tgt_cntr_offset_x_pix * angle;
    }
    else
    {
        g_ctrl_yaw_tgt = 0.0;
    }

    // Corrections for when yaw changes signs
    float abs_yaw_target = 0.0;

    if ((g_ctrl_yaw_tgt + g_mav_veh_yaw_rad) < -M_PI)
    {
        abs_yaw_target = M_PI - (std::abs(g_ctrl_yaw_tgt) - M_PI);
    }
    else if ((g_ctrl_yaw_tgt + g_mav_veh_yaw_rad) > M_PI)
    {
        abs_yaw_target = -(M_PI - (std::abs(g_ctrl_yaw_tgt) - M_PI));
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
        g_ctrl_vel_x_cmd = pid_rev.pid(Kp_x_rev, Ki_x_rev, Kd_x_rev,
                                       g_pos_err_x, ControlDim::X, g_app_dt);

        g_ctrl_vel_y_cmd = pid_rev.pid(Kp_y_rev, Ki_y_rev, Kd_y_rev,
                                       g_pos_err_y, ControlDim::Y, g_app_dt);

        g_ctrl_yaw_cmd = pid_rev.pid(Kp_yaw, Ki_yaw, Kd_yaw,
                                     g_yaw_err, ControlDim::YAW, g_app_dt);
    }
    else if (g_tgt_valid && !g_tgt_too_close)
    {
        g_ctrl_vel_x_cmd = pid_forwd.pid(Kp_x, Ki_x, Kd_x,
                                         g_pos_err_x, ControlDim::X, g_app_dt);

        g_ctrl_vel_y_cmd = pid_forwd.pid(Kp_y, Ki_y, Kd_y,
                                         g_pos_err_y, ControlDim::Y, g_app_dt);

        g_ctrl_yaw_cmd = pid_forwd.pid(Kp_yaw, Ki_yaw, Kd_yaw,
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

/********************************************************************************
 * Function: dtrmn_mod_vel_cmd
 * Description: Determine when to apply braking velocity - a modification of the
 *              velocity command to prevent overshooting the target.
 ********************************************************************************/
void dtrmn_mod_vel_cmd(void)
{
    // Braking time needed is when is 0
    if (ctrl_ebrake_vel_desired < g_veh_vel_x_est)
    {
        g_ctrl_prdtd_time_to_stop = g_app_dt * div(std::log(div(ctrl_ebrake_vel_desired, g_veh_vel_x_est)),
                                                   std::log(ctrl_arx_veh_vel_alpha));
    }
    else
    {
        g_ctrl_prdtd_time_to_stop = (float)0.0;
    }
    g_ctrl_prdtd_time_to_reach_tgt = std::abs(div(g_pos_err_x, g_tgt_vel_x_est));
    if (g_ctrl_prdtd_time_to_reach_tgt > (float)50.0)
    {
        g_ctrl_prdtd_time_to_reach_tgt = (float)50.0;
    }

    g_ctrl_apprchng_tgt = (g_tgt_vel_x_est < (float)0.0 ? true : false);
    g_ctrl_brake_cmd = (g_ctrl_apprchng_tgt &&
                        g_pos_err_x < ctrl_ebrake_dist_thresh &&
                        g_veh_vel_x_est > ctrl_ebrake_vel_thresh &&
                        g_ctrl_prdtd_time_to_reach_tgt < g_ctrl_prdtd_time_to_stop);

    g_ctrl_vel_x_cmd = (g_ctrl_brake_cmd ? (float)0.0 : g_ctrl_vel_x_cmd);

    g_sim_veh_vel_x_est = ctrl_arx_veh_vel_alpha * g_sim_veh_vel_x_est +
                          ctrl_arx_veh_vel_beta * g_ctrl_vel_x_cmd;
}

/********************************************************************************
 * Function: VelocityController
 * Description: VelocityController class constructor.
 ********************************************************************************/
VelocityController::VelocityController(void)
{
}

/********************************************************************************
 * Function: ~VelocityController
 * Description: VelocityController class destructor.
 ********************************************************************************/
VelocityController::~VelocityController(void) {}

/********************************************************************************
 * Function: init
 * Description: Initialize all planner variables.  Run once at the start
 *              of the program.
 ********************************************************************************/
bool VelocityController::init(void)
{
    get_path_params();

    g_tgt_too_close = false;
    g_pos_err_x = (float)0.0;
    g_pos_err_y = (float)0.0;
    g_ctrl_vel_x_cmd = (float)0.0;
    g_ctrl_vel_y_cmd = (float)0.0;
    g_ctrl_vel_z_cmd = (float)0.0;
    g_ctrl_yaw_tgt = (float)0.0;
    yaw_initial = (float)0.0;
    yaw_initial_latched = false;
    max_yaw = (float)0.0;
    min_yaw = (float)0.0;
    g_ctrl_yaw_cmd = (float)0.0;
    g_mav_veh_yaw_prv = (float)0.0;
    g_yaw_err = (float)0.0;
    g_veh_yaw_playback_adj = (float)0.0;
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
void VelocityController::loop(void)
{
    calc_veh_speed();
    calc_follow_error();
    calc_yaw_target_error();
    dtrmn_vel_cmd();
    dtrmn_mod_vel_cmd();
}

/********************************************************************************
 * Function: shutdown
 * Description: Function to clean up planner at the end of the program.
 ********************************************************************************/
void VelocityController::shutdown(void)
{
    // place clean up code here
}
