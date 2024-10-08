#ifdef JETSON_B01

/********************************************************************************
 * @file    follow_target.cpp
 * @author  Cameron Rose
 * @date    6/7/2023
 * @brief   Follow the target and maintain a specified x, y, z offset.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "follow_target.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/
/* Max index of <DIM> based on <PARAM> */
#define MAX_IDX_D_WIDTH 31
#define MAX_IDX_D_HEIGHT 25
#define MAX_IDX_Y_D_OFFSET 16
#define MAX_IDX_Y_PIXEL_OFFSET 13
#define MAX_IDX_DELTA_D_OFFSET 8
#define MAX_IDX_DELTA_PIXEL_OFFSET 8

/********************************************************************************
 * Object definitions
 ********************************************************************************/
DebugTerm FollowData("");
PID pid_forwd;
PID pid_rev;

bool target_too_close;
float x_error;
float y_error;
float vx_adjust;
float vy_adjust;
float vz_adjust;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/
#ifdef DEBUG_BUILD

float Kp_x;
float Ki_x;
float Kd_x;
float Kp_y;
float Ki_y;
float Kd_y;
float w1_x;
float w2_x;
float w3_x;
float w1_y;
float w2_y;
float w3_y;
float w1_z;
float w2_z;
float w3_z;
float Kp_x_rev;
float Ki_x_rev;
float Kd_x_rev;
float w1_x_rev;
float w2_x_rev;
float w3_x_rev;
float Kp_y_rev;
float Ki_y_rev;
float Kd_y_rev;
float w1_y_rev;
float w2_y_rev;
float w3_y_rev;
uint16_t vehicle_rel_height_err;
uint16_t vehicle_height_desired;
float target_height_desired;
float target_width_desired;
float x_desired;
float y_desired;

#else // RELEASE_BUILD

/* x forward */
const float Kp_x = 0.25f;
const float Ki_x = 0.00009f;
const float Kd_x = 0.00005f;
const float w1_x = 1.0f;
const float w2_x = 0.0f;
const float w3_x = 0.0f;

/* y forward */
const float Kp_y = 0.001f;
const float Ki_y = 0.00009f;
const float Kd_y = 0.00005f;
const float w1_y = 1.0f;
const float w2_y = 0.0f;
const float w3_y = 0.0f;

/* z forward */
const float w1_z = 0.0f;
const float w2_z = 0.0f;
const float w3_z = 0.0f;

/* x reverse */
const float Kp_x_rev = 0.001f;
const float Ki_x_rev = 0.00009f;
const float Kd_x_rev = 0.00005f;
const float w1_x_rev = 1.0f;
const float w2_x_rev = 0.0f;
const float w3_x_rev = 0.0f;

/* y reverse
 */
const float Kp_y_rev = 0.01f;
const float Ki_y_rev = 0.0f;
const float Kd_y_rev = 0.0001f;
const float w1_y_rev = 1.0f;
const float w2_y_rev = 0.0f;
const float w3_y_rev = 0.0f;
const uint16_t vehicle_rel_height_err = 0.0f;
const uint16_t vehicle_height_desired = 0.0f;
const float target_height_desired = 0.0f;
const float target_width_desired = 0.0f;
const float x_desired = 1.0f; // Make const in the end
const float y_desired = 0.0f; // Make const in the ends

#endif // DEBUG_BUILD

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: Follow
 * Description: Follow class constructor.
 ********************************************************************************/
Follow::Follow(void) {};

/********************************************************************************
 * Function: ~Follow
 * Description: Follow class destructor.
 ********************************************************************************/
Follow::~Follow(void) {};

/********************************************************************************
 * Function: get_control_params
 * Description: Read follow control parameters from a json or other file type.
 ********************************************************************************/
void Follow::get_control_params(void)
{
#ifdef DEBUG_BUILD

    Parameters veh_params("../params.json");
    // Accessing Vel_PID_x parameters
    Kp_x = veh_params.get_float_param("Vel_PID_x", "Kp");
    Ki_x = veh_params.get_float_param("Vel_PID_x", "Ki");
    Kd_x = veh_params.get_float_param("Vel_PID_x", "Kd");
    w1_x = veh_params.get_float_param("Vel_PID_x", "w1");
    w2_x = veh_params.get_float_param("Vel_PID_x", "w2");
    w3_x = veh_params.get_float_param("Vel_PID_x", "w3");

    // Accessing Vel_PID_y parameters
    Kp_y = veh_params.get_float_param("Vel_PID_y", "Kp");
    Ki_y = veh_params.get_float_param("Vel_PID_y", "Ki");
    Kd_y = veh_params.get_float_param("Vel_PID_y", "Kd");
    w1_y = veh_params.get_float_param("Vel_PID_y", "w1");
    w2_y = veh_params.get_float_param("Vel_PID_y", "w2");
    w3_y = veh_params.get_float_param("Vel_PID_y", "w3");

    // Accessing Vel_PID_x parameters for reverse movement
    Kp_x_rev = veh_params.get_float_param("Vel_PID_x_reverse", "Kp");
    Ki_x_rev = veh_params.get_float_param("Vel_PID_x_reverse", "Ki");
    Kd_x_rev = veh_params.get_float_param("Vel_PID_x_reverse", "Kd");
    w1_x_rev = veh_params.get_float_param("Vel_PID_x_reverse", "w1");
    w2_x_rev = veh_params.get_float_param("Vel_PID_x_reverse", "w2");
    w3_x_rev = veh_params.get_float_param("Vel_PID_x_reverse", "w3");

    // Accessing Vel_PID_y parameters for reverse movment
    Kp_y_rev = veh_params.get_float_param("Vel_PID_y_reverse", "Kp");
    Ki_y_rev = veh_params.get_float_param("Vel_PID_y_reverse", "Ki");
    Kd_y_rev = veh_params.get_float_param("Vel_PID_y_reverse", "Kd");
    w1_y_rev = veh_params.get_float_param("Vel_PID_y_reverse", "w1");
    w2_y_rev = veh_params.get_float_param("Vel_PID_y_reverse", "w2");
    w3_y_rev = veh_params.get_float_param("Vel_PID_y_reverse", "w3");

#endif // DEBUG_BUILD
}

/********************************************************************************
 * Function: calc_follow_error
 * Description: Calculate the error between the desired target offset and the
 *              target's actual offset.
 ********************************************************************************/
void Follow::calc_follow_error(void)
{

    // Parameters target_params("../params.json");

    // x_desired = target_params.get_float_param("Target", "Desired_X_offset");

    if (d_object < 0.001f)
    {
        x_error = 0.0f;
    }
    else
    {
        x_error = x_object - x_desired;
    }

    y_error = y_object - y_desired;
}

/********************************************************************************
 * Function: init
 * Description: Initialize all follow target variables.  Run once at the start
 *              of the program.
 ********************************************************************************/
bool Follow::init(void)
{
    target_too_close = false;
    x_error = 0.0f;
    y_error = 0.0f;
    vx_adjust = 0.0f;
    vy_adjust = 0.0f;
    vz_adjust = 0.0f;

    return true;
}

/********************************************************************************
 * Function: loop
 * Description: Return control parameters for the vehicle to follow a designated
 *              target at a distance.
 ********************************************************************************/
void Follow::loop(void)
{
    get_control_params();
    calc_follow_error();

    target_too_close = (x_error < 0.0);

    if (target_identified && target_too_close)
    {
        vx_adjust = pid_rev.pid_controller_3d(Kp_x_rev, Ki_x_rev, Kd_x_rev,
                                              x_error, 0.0, 0.0,
                                              w1_x_rev, 0.0, 0.0, CONTROL_DIM::X);
        vy_adjust = pid_rev.pid_controller_3d(Kp_y_rev, Ki_y_rev, Kd_y_rev,
                                              y_error, 0.0, 0.0,
                                              w1_y_rev, 0.0, 0.0, CONTROL_DIM::Y);
    }
    else if (target_identified && !target_too_close)
    {
        vx_adjust = pid_forwd.pid_controller_3d(Kp_x, Ki_x, Kd_x,
                                                x_error, 0.0, 0.0,
                                                w1_x, 0.0, 0.0, CONTROL_DIM::X);
        vy_adjust = pid_forwd.pid_controller_3d(Kp_y, Ki_y, Kd_y,
                                                y_error, 0.0, 0.0,
                                                w1_y, 0.0, 0.0, CONTROL_DIM::Y);
    }
    else
    {
        vx_adjust = 0.0f;
        vy_adjust = 0.0f;
    }
}

#endif // JETSON_B01