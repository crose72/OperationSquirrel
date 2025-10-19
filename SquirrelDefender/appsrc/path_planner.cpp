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
#include "path_planner.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/
#define NUM_VX_DECEL_STEPS 10

/********************************************************************************
 * Object definitions
 ********************************************************************************/
PID pid_forwd;
PID pid_rev;
PID pid_yaw;

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

float vx_cmd_max_allowed = (float)10.0;   // max allowed velocity
float vx_cmd_ramp_step_size = (float)0.5; // m/s²
float vx_cmd_max_ramp_rate = (float)2.5;  // m/s²
float vx_cmd_diverge_sensitivity = (float)0.05;
float vx_cmd_parabolic_easing_thresh = (float)1.0;
float vx_cmd_parabolic_easing_max = (float)1.0;
float vx_cmd_filt_coef = (float)0.05;

const float camera_half_fov = (float)0.7423; // half of 83 degree FOV camera

const float vx_decel_profile_start_idx[NUM_VX_DECEL_STEPS] = {
    4.500, 2.757, 1.689, 1.034, 0.634, 0.389, 0.239, 0.146, 0.089, 0.054};

const float vx_decel_profile[NUM_VX_DECEL_STEPS] = {
    4.500, 2.757, 1.689, 1.034, 0.634, 0.389, 0.239, 0.146, 0.089, 0.054};

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

    vx_cmd_max_allowed = follow_control.get_float_param("Follow_Params", "CMD_vel_max_cmd");
    vx_cmd_max_ramp_rate = follow_control.get_float_param("Follow_Params", "CMD_vel_max_ramp_rate");
    vx_cmd_ramp_step_size = follow_control.get_float_param("Follow_Params", "CMD_vel_ramp_step_size");
    vx_cmd_diverge_sensitivity = follow_control.get_float_param("Follow_Params", "CMD_vel_diverge_sensitivity");
    vx_cmd_parabolic_easing_thresh = follow_control.get_float_param("Follow_Params", "CMD_vel_parabolic_eas_thresh");
    vx_cmd_parabolic_easing_max = follow_control.get_float_param("Follow_Params", "CMD_vel_parabolic_eas_max_cmd");

    vx_cmd_filt_coef = follow_control.get_float_param("Follow_Params", "CMD_vel_X_filt_coef");
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
        // Yaw has a max value of PI and a min value of - PI.
        // If the sum of the yaw target and the initial latched yaw state of the
        // vehicle results in a value < -PI or > PI then the yaw value will have
        // to account for that, since when yaw becomes > PI or < -PI it rolls over
        // positive sign to negative or visa versa.
        float abs_min_yaw_temp = yaw_initial - camera_half_fov;

        if (abs_min_yaw_temp >= -PI)
        {
            min_yaw = abs_min_yaw_temp;
        }
        else
        {
            min_yaw = PI - (std::abs(abs_min_yaw_temp) - PI);
        }

        float abs_max_yaw_temp = yaw_initial + camera_half_fov;

        if (abs_max_yaw_temp <= PI)
        {
            max_yaw = abs_max_yaw_temp;
        }
        else
        {
            max_yaw = -(PI - (abs_max_yaw_temp - PI));
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

    // Corrections for when yaw changes signs
    float abs_yaw_target = 0.0;

    if ((g_yaw_target + g_mav_veh_yaw) < -PI)
    {
        abs_yaw_target = PI - (std::abs(g_yaw_target) - PI);
    }
    else if ((g_yaw_target + g_mav_veh_yaw) > PI)
    {
        abs_yaw_target = -(PI - (std::abs(g_yaw_target) - PI));
    }

    float abs_max_yaw_temp = yaw_initial + camera_half_fov;

    if (abs_max_yaw_temp <= PI)
    {
        max_yaw = abs_max_yaw_temp;
    }
    else
    {
        max_yaw = -(PI - (abs_max_yaw_temp - PI));
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
                the desired target offset and the actual target offset.
 ********************************************************************************/
void dtrmn_vel_cmd(void)
{
    g_target_too_close = (g_x_error < 0.0);

    if (g_target_valid && g_target_too_close)
    {
        g_vx_adjust = pid_rev.pid3(Kp_x_rev, Ki_x_rev, Kd_x_rev,
                                   g_x_error, 0.0, 0.0,
                                   w1_x_rev, 0.0, 0.0, ControlDim::X, g_dt);

        g_vy_adjust = pid_rev.pid3(Kp_y_rev, Ki_y_rev, Kd_y_rev,
                                   g_y_error, 0.0, 0.0,
                                   w1_y_rev, 0.0, 0.0, ControlDim::Y, g_dt);
        g_yaw_adjust = pid_yaw.pid3(Kp_yaw, Ki_yaw, Kd_yaw,
                                    g_yaw_target_error, 0.0, 0.0,
                                    w1_yaw, 0.0, 0.0, ControlDim::YAW, g_dt);
    }
    else if (g_target_valid && !g_target_too_close)
    {
        g_vx_adjust = pid_forwd.pid3(Kp_x, Ki_x, Kd_x,
                                     g_x_error, 0.0, 0.0,
                                     w1_x, 0.0, 0.0, ControlDim::X, g_dt);

        g_vy_adjust = pid_forwd.pid3(Kp_y, Ki_y, Kd_y,
                                     g_y_error, 0.0, 0.0,
                                     w1_y, 0.0, 0.0, ControlDim::Y, g_dt);
        g_yaw_adjust = pid_yaw.pid3(Kp_yaw, Ki_yaw, Kd_yaw,
                                    g_yaw_target_error, 0.0, 0.0,
                                    w1_yaw, 0.0, 0.0, ControlDim::YAW, g_dt);
    }
    else
    {
        g_vx_adjust = (float)0.0;
        g_vy_adjust = (float)0.0;
        g_yaw_adjust = (float)0.0;
    }

    target_valid_last_cycle = g_target_valid;
    vx_adjust_prv = g_vx_adjust;
    x_error_prv = g_x_error;
}

/********************************************************************************
 * Function: PathPlanner
 * Description: PathPlanner class constructor.
 ********************************************************************************/
PathPlanner::PathPlanner(void) {};

/********************************************************************************
 * Function: ~PathPlanner
 * Description: PathPlanner class destructor.
 ********************************************************************************/
PathPlanner::~PathPlanner(void) {};

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
    vx_decel_prof_idx = (float)0.0;
    decel_profile_active = false;
    profile_sign = (int)1;
    target_valid_last_cycle = false;

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