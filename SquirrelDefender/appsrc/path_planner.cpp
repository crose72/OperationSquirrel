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
float g_mav_veh_yaw_adjusted;
float g_target_cntr_offset_x_filt;
float target_cntr_offset_x_prv;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/
// Eventually these should all be 
/* x forward */
 float Kp_x = 0.5f;
 float Ki_x = 0.009f;
 float Kd_x = 0.1f;
 float w1_x = 1.0f;
 float w2_x = 0.0f;
 float w3_x = 0.0f;

/* y forward */
 float Kp_y = 0.03;
 float Ki_y = 0.00f;
 float Kd_y = 0.001f;
 float w1_y = 1.0f;
 float w2_y = 0.0f;
 float w3_y = 0.0f;

/* z forward */
 float w1_z = 0.0f;
 float w2_z = 0.0f;
 float w3_z = 0.0f;

/* x reverse */
 float Kp_x_rev = 0.0;
 float Ki_x_rev = 0.0;
 float Kd_x_rev = 0.0;
 float w1_x_rev = 1.0f;
 float w2_x_rev = 0.0f;
 float w3_x_rev = 0.0f;

/* y reverse */
 float Kp_y_rev = 0.005;
 float Ki_y_rev = 0.0f;
 float Kd_y_rev = 0.0;
 float w1_y_rev = 1.0f;
 float w2_y_rev = 0.0f;
 float w3_y_rev = 0.0f;

float Kp_yaw = 0.08;
float Ki_yaw = 0.0;
float Kd_yaw = 0.00005;
float w1_yaw = 1.0;
float w2_yaw = 0.0;
float w3_yaw = 0.0;

float x_desired = 4.0f;
float y_desired = 0.0f;

const float camera_half_fov = 0.7423; // half of 83 degree FOV camera

/********************************************************************************
 * Function definitions
 ********************************************************************************/
void get_control_params(void);
void calc_follow_error(void);
void calc_yaw_target_error(void);
void dtrmn_follow_vector(void);

/********************************************************************************
 * Function: get_control_params
 * Description: Read follow control parameters from a json or other file type.
 ********************************************************************************/
void get_control_params(void)
{
    ParamReader follow_control("../params.json");

    // Accessing Vel_PID_x parameters
    Kp_x = follow_control.get_float_param("Vel_PID_x", "Kp");
    Ki_x = follow_control.get_float_param("Vel_PID_x", "Ki");
    Kd_x = follow_control.get_float_param("Vel_PID_x", "Kd");
    w1_x = follow_control.get_float_param("Vel_PID_x", "w1");
    w2_x = follow_control.get_float_param("Vel_PID_x", "w2");
    w3_x = follow_control.get_float_param("Vel_PID_x", "w3");

    // Accessing Vel_PID_y parameters
    Kp_y = follow_control.get_float_param("Vel_PID_y", "Kp");
    Ki_y = follow_control.get_float_param("Vel_PID_y", "Ki");
    Kd_y = follow_control.get_float_param("Vel_PID_y", "Kd");
    w1_y = follow_control.get_float_param("Vel_PID_y", "w1");
    w2_y = follow_control.get_float_param("Vel_PID_y", "w2");
    w3_y = follow_control.get_float_param("Vel_PID_y", "w3");

    // Accessing Vel_PID_x parameters for reverse movement
    Kp_x_rev = follow_control.get_float_param("Vel_PID_x_reverse", "Kp");
    Ki_x_rev = follow_control.get_float_param("Vel_PID_x_reverse", "Ki");
    Kd_x_rev = follow_control.get_float_param("Vel_PID_x_reverse", "Kd");
    w1_x_rev = follow_control.get_float_param("Vel_PID_x_reverse", "w1");
    w2_x_rev = follow_control.get_float_param("Vel_PID_x_reverse", "w2");
    w3_x_rev = follow_control.get_float_param("Vel_PID_x_reverse", "w3");

    // Accessing Vel_PID_y parameters for reverse movment
    Kp_y_rev = follow_control.get_float_param("Vel_PID_y_reverse", "Kp");
    Ki_y_rev = follow_control.get_float_param("Vel_PID_y_reverse", "Ki");
    Kd_y_rev = follow_control.get_float_param("Vel_PID_y_reverse", "Kd");
    w1_y_rev = follow_control.get_float_param("Vel_PID_y_reverse", "w1");
    w2_y_rev = follow_control.get_float_param("Vel_PID_y_reverse", "w2");
    w3_y_rev = follow_control.get_float_param("Vel_PID_y_reverse", "w3");

    // Accessing Yaw PID parameters
    Kp_yaw = follow_control.get_float_param("Yaw_PID", "Kp");
    Ki_yaw = follow_control.get_float_param("Yaw_PID", "Ki");
    Kd_yaw = follow_control.get_float_param("Yaw_PID", "Kd");
    w1_yaw = follow_control.get_float_param("Yaw_PID", "w1");
    w2_yaw = follow_control.get_float_param("Yaw_PID", "w2");
    w3_yaw = follow_control.get_float_param("Yaw_PID", "w3");

    // Follow params
    x_desired = follow_control.get_float_param("Target", "Desired_X_offset");
    y_desired = follow_control.get_float_param("Target", "Desired_Y_offset");
}

/********************************************************************************
 * Function: calc_follow_error
 * Description: Calculate the error between the desired target offset and the
 *              target's actual offset.
 ********************************************************************************/
void calc_follow_error(void)
{
    float x_error_abs;
    float y_error_abs;

    ParamReader plan_params("../params.json");

    float filter_coeff = plan_params.get_float_param("Detection_tracking", "BBox_Filt_Coeff");

    g_target_cntr_offset_x_filt = low_pass_filter(g_target_cntr_offset_x, target_cntr_offset_x_prv, filter_coeff);
    target_cntr_offset_x_prv = g_target_cntr_offset_x_filt;

    if (g_target_loc_data_ok)
    {

        if (g_target_cntr_offset_x_filt < (float)(-50.0))
        {
            g_x_error = g_target_cntr_offset_x_filt;
        }
        else if (g_target_cntr_offset_x_filt > (float)50.0)
        {
            g_x_error = g_target_cntr_offset_x_filt;
        }
        else
        {
            g_x_error = (float)0.0;
        }
        
        if (g_target_cntr_offset_y < (float)(-50.0))
        {
            g_y_error = g_target_cntr_offset_x;
        }
        else if (g_target_cntr_offset_y < (float)50.0)
        {
            g_y_error = g_target_cntr_offset_y;
        }
        else
        {
            g_y_error = (float)0.0;
        }
    }
}

/*
void calc_follow_error(void)
{
    if (g_x_target_ekf < 0.001f)
    {
        g_x_error = 0.0f;
    }
    else
    {
        g_x_error = g_x_target_ekf - x_desired;
    }

    g_y_error = g_y_target_ekf - y_desired;
}
*/

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

    g_mav_veh_yaw_adjusted = g_mav_veh_yaw - yaw_initial;

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
        g_yaw_target_error = g_yaw_target - g_mav_veh_yaw_adjusted;
    }
    else
    {
        g_yaw_target_error = g_yaw_target;
    }
}

/********************************************************************************
 * Function: dtrmn_follow_vector
 * Description: Determine the follow vector based on the vehicle's error between
                the desired target offset and the actual target offset.
 ********************************************************************************/
void dtrmn_follow_vector(void)
{
    g_target_too_close = (g_x_error < 0.0);

    /*
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
    else
    */
    if (g_target_valid && !g_target_too_close)
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
        g_vx_adjust = 0.0f;
        g_vy_adjust = 0.0f;
        g_yaw_adjust = 0.0f;
    }
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
    g_target_too_close = false;
    g_x_error = 0.0;
    g_y_error = 0.0;
    g_vx_adjust = 0.0;
    g_vy_adjust = 0.0;
    g_vz_adjust = 0.0;
    g_yaw_target = 0.0;
    yaw_initial = 0.0;
    yaw_initial_latched = false;
    max_yaw = 0.0;
    min_yaw = 0.0;
    g_yaw_adjust = 0.0;
    g_mav_veh_yaw_prv = 0.0;
    g_yaw_target_error = 0.0;
    g_mav_veh_yaw_adjusted = 0.0;
    g_target_cntr_offset_x_filt = (float)0.0;
    target_cntr_offset_x_prv = (float)0.0;

    get_control_params();

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
    dtrmn_follow_vector();
}

/********************************************************************************
 * Function: shutdown
 * Description: Function to clean up planner at the end of the program.
 ********************************************************************************/
void PathPlanner::shutdown(void)
{
    // place clean up code here
}