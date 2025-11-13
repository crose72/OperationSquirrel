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
float vxy_max_allowed_jerk = 8.0f;            // m/s^3 for x and y (jerk cap)

/********************************************************************************
 * Function definitions
 ********************************************************************************/
void get_path_params(void);
void calc_follow_error(void);
void calc_yaw_target_error(void);
void dtrmn_vel_cmd(void);

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

    // Accessing Vel_PID_y parameters
    Kp_y = follow_control.get_float_param("PID_vy_forward", "Kp");
    Ki_y = follow_control.get_float_param("PID_vy_forward", "Ki");
    Kd_y = follow_control.get_float_param("PID_vy_forward", "Kd");

    // Accessing Vel_PID_x parameters for reverse movement
    Kp_x_rev = follow_control.get_float_param("PID_vx_reverse", "Kp");
    Ki_x_rev = follow_control.get_float_param("PID_vx_reverse", "Ki");
    Kd_x_rev = follow_control.get_float_param("PID_vx_reverse", "Kd");

    // Accessing Vel_PID_y parameters for reverse movment
    Kp_y_rev = follow_control.get_float_param("PID_vy_reverse", "Kp");
    Ki_y_rev = follow_control.get_float_param("PID_vy_reverse", "Ki");
    Kd_y_rev = follow_control.get_float_param("PID_vy_reverse", "Kd");

    // Accessing Yaw PID parameters
    Kp_yaw = follow_control.get_float_param("PID_yaw", "Kp");
    Ki_yaw = follow_control.get_float_param("PID_yaw", "Ki");
    Kd_yaw = follow_control.get_float_param("PID_yaw", "Kd");

    // Follow params
    x_desired = follow_control.get_float_param("Follow_Params", "Desired_X_offset");
    y_desired = follow_control.get_float_param("Follow_Params", "Desired_Y_offset");

    vxy_cmd_max_allowed_accel = follow_control.get_float_param("Follow_Params", "CMD_vxy_max_allowed_accel");
    vxy_max_allowed_jerk = follow_control.get_float_param("Follow_Params", "CMD_vxy_max_allowed_jerk");
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
        g_vx_adjust = pid_rev.pid(Kp_x_rev, Ki_x_rev, Kd_x_rev,
                                  g_x_error, ControlDim::X, g_dt);

        g_vy_adjust = pid_rev.pid(Kp_y_rev, Ki_y_rev, Kd_y_rev,
                                  g_y_error, ControlDim::Y, g_dt);

        g_yaw_adjust = pid_rev.pid(Kp_yaw, Ki_yaw, Kd_yaw,
                                   g_yaw_target_error, ControlDim::YAW, g_dt);
    }
    else if (g_target_valid && !g_target_too_close)
    {
        g_vx_adjust = pid_forwd.pid(Kp_x, Ki_x, Kd_x,
                                    g_x_error, ControlDim::X, g_dt);

        g_vy_adjust = pid_forwd.pid(Kp_y, Ki_y, Kd_y,
                                    g_y_error, ControlDim::Y, g_dt);

        g_yaw_adjust = pid_forwd.pid(Kp_yaw, Ki_yaw, Kd_yaw,
                                     g_yaw_target_error, ControlDim::YAW, g_dt);
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

    target_valid_last_cycle = g_target_valid;
    vx_adjust_prv = g_vx_adjust;
    x_error_prv = g_x_error;
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
                           /*j_max_xy=*/vxy_max_allowed_jerk);
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
    calc_follow_error();
    calc_yaw_target_error();
    dtrmn_vel_cmd();
}

/********************************************************************************
 * Function: shutdown
 * Description: Function to clean up planner at the end of the program.
 ********************************************************************************/
void PathPlanner::shutdown(void)
{
    // place clean up code here
}
