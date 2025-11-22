#ifdef ENABLE_CV

/********************************************************************************
 * @file    target_localization.cpp
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   TargetLocalization the x, y, z offset of the target relative to the vehicle.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "target_localization.h"
#include "mav_data_hub.h"
#include "target_tracking.h"
#include "video_io.h"
#include "param_reader.h"
#include "interpolate.h"
#include "signal_processing.h"
#include "time_calc.h"
#include "kf.h"
#include "timer.h"
#include <cmath>
#include <algorithm>

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
Timer target_data_useful_dbc;
float g_tgt_los_dist_from_pix_height;
float g_tgt_los_dist_from_pix_width;
float g_tgt_pos_x_meas;
float g_tgt_pos_y_meas;
float g_tgt_pos_z_meas;
float g_tgt_los_dist_meas;
float g_cam0_delta_angle_rad; //
float g_cam0_angle_rad;       // Angle of the camera relative to the ground, compensating for pitch
float g_tgt_pos_x_delta;
float g_tgt_pos_z_delta;
float g_cam0_comp_angle_rad;
float x_target_prv;
float y_target_prv;
bool y_target_hyst_actv;
float y_target_latched;
bool loc_target_valid_prv;
bool g_tgt_meas_valid;
float loc_target_center_x_prv;
float loc_target_center_y_prv;
float g_tgt_pos_x_est;
float g_tgt_pos_y_est;
float g_tgt_vel_x_est;
float g_tgt_vel_y_est;
float g_tgt_acc_x_est;
float g_tgt_acc_y_est;
float x_target_ekf_prv;
float y_target_ekf_prv;
float camera_comp_angle_abs;
float g_cam0_fov_height;
float g_cam0_m_per_pix;
float g_tgt_cntr_offset_x_m;
float g_cam0_los_m;
static bool target_valid_prv;
bool kf_loc_reset;
bool target_data_useful_prv;

// Buffers for moving average of target centroid
std::vector<float> target_y_pix_hist4avg;
std::vector<float> target_x_pix_hist4avg;
int x_pix_window;
int y_pix_window;
float g_tgt_cntr_offset_x_pix_filt;
float g_tgt_cntr_offset_y_pix_filt;
int y_buffer_idx4avg;
int x_buffer_idx4avg;
float y_sum;
float x_sum;

KF kf_loc;           // Kalman Filter instance
arma::mat A_loc;     // System dynamics matrix
arma::mat B_loc;     // Control matrix (if needed, otherwise identity)
arma::mat H_loc;     // Observation matrix
arma::mat Q_loc;     // Process noise covariance
arma::mat R_loc;     // Measurement noise covariance
arma::mat P_loc;     // Estimate error covariance
arma::colvec x0_loc; // Initial state
arma::colvec u0_loc; // Observed initial measurements

std::chrono::milliseconds g_target_lost_dbc_ms;
std::chrono::milliseconds target_lost_dbc_reset_ms;
float target_lost_dbc_reset_sec = (float)0.0;
bool g_tgt_lost;
bool target_is_lost_prv;
float g_tgt_lost_dbc_sec;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/
const float delta_offset_d[MAX_IDX_DELTA_D_OFFSET] = {
    0.00f, 1.00f, 2.00f, 3.00f, 4.00f, 5.00f, 6.00f, 7.00f};

const float delta_offset_pixels[MAX_IDX_DELTA_PIXEL_OFFSET] = {
    0.00f, 50.00f, 100.00f, 150.00f, 200.00f, 250.00f, 300.00f, 350.00f};

// Define the 2D array for target y values
const float delta_offset[MAX_IDX_DELTA_PIXEL_OFFSET][MAX_IDX_DELTA_D_OFFSET] = {
    {0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f},
    {0.00f, 0.08f, 0.11f, 0.15f, 0.20f, 0.27f, 0.30f, 0.40f},
    {0.00f, 0.17f, 0.23f, 0.30f, 0.40f, 0.53f, 0.60f, 0.80f},
    {0.00f, 0.25f, 0.34f, 0.45f, 0.60f, 0.80f, 0.90f, 1.20f},
    {0.00f, 0.34f, 0.45f, 0.60f, 0.80f, 1.06f, 1.20f, 1.60f},
    {0.00f, 0.42f, 0.56f, 0.75f, 1.00f, 1.33f, 1.50f, 2.00f},
    {0.00f, 0.51f, 0.68f, 0.90f, 1.20f, 1.60f, 1.80f, 2.39f},
    {0.00f, 0.59f, 0.79f, 1.05f, 1.40f, 1.86f, 2.10f, 2.79f}};

// Kalman filter params
const float dt_kf_loc = 0.05;

// H_loc (Observation matrix, 2x6)
float H_loc_00 = 1.0f;
float H_loc_11 = 1.0f;

// Q_loc (Process noise covariance diagonals, 6x6)
float Q_loc_00 = 0.01f;
float Q_loc_11 = 0.0001f;
float Q_loc_22 = 0.00001f;
float Q_loc_33 = 0.00001f;
float Q_loc_44 = 0.00001f;
float Q_loc_55 = 0.00001f;

// R_loc (Measurement noise covariance diagonals, 2x2)
float R_loc_00 = 10.0f;
float R_loc_11 = 40.0f;

// P_loc (Estimate covariance diagonals, 6x6)
float P_loc_00 = 0.1f;
float P_loc_11 = 0.1f;
float P_loc_22 = 0.1f;
float P_loc_33 = 0.1f;
float P_loc_44 = 0.1f;
float P_loc_55 = 0.1f;

const int n_states = 6;
const int n_meas = 2;
float known_obj_heigh_all_dist = (float)1.778; // Height of known object at all distances
// Coefficients and power for the equation relating target height to distance
// from the camera in the forward direction.  Equation solved so the input
// is target size (bounding box in pixels) and output is target estimated
// distance (in meters). The equation is: dist = (bbox dim / coef) ^ (1 / pow)
float pix_height_x_coef = (float)1680.557;
float pix_height_x_pow = (float)(-0.863);
float pix_width_x = (float)540.456;
float pix_width_x_pow = (float)(-0.758);
float target_det_edge_of_frame_buffer = (float)50.0;
float min_target_bbox_area = (float)3500.0;

/********************************************************************************
 * Function definitions
 ********************************************************************************/
void get_localization_params(void);
void init_kf_loc(void);
void calc_fov(void);
void dtrmn_target_loc_img(void);
void dtrmn_target_loc_real(void);
void update_target_loc_real(void);

/********************************************************************************
 * Function: get_localization_params
 * Description: Calibratable parameters for target localization.
 ********************************************************************************/
void get_localization_params(void)
{
    ParamReader localization_params("../params.json");

    // H_loc (Observation matrix, 2x6)
    H_loc_00 = localization_params.get_float_param("target_loc_params.h_pos_x_00");
    H_loc_11 = localization_params.get_float_param("target_loc_params.h_pos_y_11");

    // Q_loc (Process noise covariance diagonals, 6x6)
    Q_loc_00 = localization_params.get_float_param("target_loc_params.q_pos_x_00");
    Q_loc_11 = localization_params.get_float_param("target_loc_params.q_pos_y_11");
    Q_loc_22 = localization_params.get_float_param("target_loc_params.q_vel_x_22");
    Q_loc_33 = localization_params.get_float_param("target_loc_params.q_vel_y_33");
    Q_loc_44 = localization_params.get_float_param("target_loc_params.q_acc_x_44");
    Q_loc_55 = localization_params.get_float_param("target_loc_params.q_acc_y_55");

    // R_loc (Measurement noise covariance diagonals, 2x2)
    R_loc_00 = localization_params.get_float_param("target_loc_params.r_pos_x_00");
    R_loc_11 = localization_params.get_float_param("target_loc_params.r_pos_y_11");

    // P_loc (Estimate covariance diagonals, 6x6)
    P_loc_00 = localization_params.get_float_param("target_loc_params.p_pos_x_00");
    P_loc_11 = localization_params.get_float_param("target_loc_params.p_pos_y_11");
    P_loc_22 = localization_params.get_float_param("target_loc_params.p_vel_x_22");
    P_loc_33 = localization_params.get_float_param("target_loc_params.p_vel_y_33");
    P_loc_44 = localization_params.get_float_param("target_loc_params.p_acc_x_44");
    P_loc_55 = localization_params.get_float_param("target_loc_params.p_acc_y_55");

    // Target centroid moving average and curve fit window sizes
    x_pix_window = localization_params.get_int_param("target_loc_params.target_bbox_center_x_history_size");
    y_pix_window = localization_params.get_int_param("target_loc_params.target_bbox_center_y_history_size");

    pix_height_x_coef = localization_params.get_float_param("target_loc_params.target_los_height_eq_coef");
    pix_height_x_pow = localization_params.get_float_param("target_loc_params.target_los_height_eq_pow");
    pix_width_x = localization_params.get_float_param("target_loc_params.target_los_width_eq_coef");
    pix_width_x_pow = localization_params.get_float_param("target_loc_params.target_los_width_eq_pow");
    known_obj_heigh_all_dist = localization_params.get_float_param("target_loc_params.known_object_height_all_dist");
    target_det_edge_of_frame_buffer = localization_params.get_float_param("target_loc_params.target_bbox_min_dist_from_edge");
    min_target_bbox_area = localization_params.get_float_param("target_loc_params.target_bbox_min_area");
    int target_lost_dbc_reset_val = localization_params.get_float_param("target_loc_params.target_bbox_lost_min_time_ms");
    target_lost_dbc_reset_sec = (float)target_lost_dbc_reset_val / (float)1000.0;
    target_lost_dbc_reset_ms = static_cast<std::chrono::milliseconds>(target_lost_dbc_reset_val);
}

/********************************************************************************
 * Function: init_kf_loc
 * Description: Initialize kalman filter for target location estimation.
 ********************************************************************************/
void init_kf_loc(void)
{
    x_target_ekf_prv = 0.0;
    y_target_ekf_prv = 0.0;
    g_tgt_pos_x_est = 0.0;
    g_tgt_pos_y_est = 0.0;
    g_tgt_vel_x_est = 0.0;
    g_tgt_vel_y_est = 0.0;
    g_tgt_acc_x_est = 0.0;
    g_tgt_acc_y_est = 0.0;

    // Resize matrices and set to zero initially
    A_loc.set_size(n_states, n_states);
    A_loc.zeros();
    B_loc.set_size(n_states, 1);
    B_loc.zeros();
    H_loc.set_size(n_meas, n_states);
    H_loc.zeros();
    Q_loc.set_size(n_states, n_states);
    Q_loc.zeros();
    R_loc.set_size(n_meas, n_meas);
    R_loc.zeros();
    P_loc.set_size(n_states, n_states);
    P_loc.zeros();

    // Define system dynamics (State transition matrix A_loc)
    A_loc << 1 << 0 << dt_kf_loc << 0 << 0.5 * pow(dt_kf_loc, 2) << 0 << arma::endr
          << 0 << 1 << 0 << dt_kf_loc << 0 << 0.5 * pow(dt_kf_loc, 2) << arma::endr
          << 0 << 0 << 1 << 0 << dt_kf_loc << 0 << arma::endr
          << 0 << 0 << 0 << 1 << 0 << dt_kf_loc << arma::endr
          << 0 << 0 << 0 << 0 << 1 << 0 << arma::endr
          << 0 << 0 << 0 << 0 << 0 << 1 << arma::endr;

    // Observation matrix (Only x and y are observed)
    arma::mat H_loc = arma::zeros<arma::mat>(2, 6);
    H_loc(0, 0) = H_loc_00;
    H_loc(1, 1) = H_loc_11;

    // Model covariance matrix Q_loc (process noise)
    arma::mat Q_loc = arma::zeros<arma::mat>(6, 6);
    Q_loc(0, 0) = Q_loc_00;
    Q_loc(1, 1) = Q_loc_11;
    Q_loc(2, 2) = Q_loc_22;
    Q_loc(3, 3) = Q_loc_33;
    Q_loc(4, 4) = Q_loc_44;
    Q_loc(5, 5) = Q_loc_55;

    // Measurement covariance matrix R_loc (sensor noise)
    arma::mat R_loc = arma::zeros<arma::mat>(2, 2);
    R_loc(0, 0) = R_loc_00;
    R_loc(1, 1) = R_loc_11;

    // Estimate covariance matrix P_loc (initial trust in the estimate)
    arma::mat P_loc = arma::zeros<arma::mat>(6, 6);
    P_loc(0, 0) = P_loc_00;
    P_loc(1, 1) = P_loc_11;
    P_loc(2, 2) = P_loc_22;
    P_loc(3, 3) = P_loc_33;
    P_loc(4, 4) = P_loc_44;
    P_loc(5, 5) = P_loc_55;

    // Initialize Kalman Filter
    kf_loc.InitSystem(A_loc, B_loc, H_loc, Q_loc, R_loc);

    // Set initial state (assuming stationary at origin)
    x0_loc.set_size(n_states);
    x0_loc.zeros(); // Start with zero velocity and acceleration
    kf_loc.InitSystemState(x0_loc);

    // Resize and reset u0_loc
    u0_loc.set_size(n_meas);
    u0_loc.zeros();
}

/********************************************************************************
 * Function: calc_fov
 * Description: Calculate the field of view of the camera in meters at the
 *              given vehicle height above the ground, and camera angle
 *              relative to the ground.
 ********************************************************************************/
void calc_fov(void)
{
    // Calculate the line of sight distance of the camera
    if ((g_cam0_comp_angle_rad + g_mav_veh_pitch_rad) < (float)0.001)
    {
        g_cam0_los_m = (float)0.0;
    }
    else
    {
        g_cam0_los_m = std::fabs(g_mav_veh_pos_ned_z) / cosf(g_cam0_comp_angle_rad + g_mav_veh_pitch_rad);
    }

    // Calculate the pixel height of a known fixed object at given line of
    // sight distance from the camera.
    // The object isn't there, this is an intermediate calculation to determine
    // how many pixels correspond to a meter at the current drone height.

    float ghost_object_height = (float)0.0;

    // Height of a known object at given distance
    if (g_cam0_los_m > (float)0.0)
    {
        float ghost_object_height = pix_height_x_coef * powf(g_cam0_los_m, pix_height_x_pow);
    }
    // How many meters a single pixel represents at the line of sight distance
    // TODO: When to use actual vs ghost object height?
    // Uses "ghost object" height - the height of the known object at the LOS distance
    // g_cam0_m_per_pix = known_obj_heigh_all_dist / ghost_object_height;
    // Currently gonna use the target actual height
    // Only compute m_per_pixel if we have a valid bbox height
    if (g_tgt_height_pix > (float)0.0f)
    {
        g_cam0_m_per_pix = known_obj_heigh_all_dist / g_tgt_height_pix;
        g_cam0_fov_height = g_cam0_m_per_pix * g_cam0_img_height_px;
    }
    else
    {
        // No detection yet – use last-known or default
        // Avoid NaN and preserve stability
        g_cam0_m_per_pix = g_cam0_m_per_pix; // or keep previous
        // OR safer: use a sane default from params
        // g_cam0_m_per_pix = default_m_per_pix;

        // Same for FOV height – maintain previous stable value
        // g_cam0_fov_height = default_fov_m;
    }
}

/********************************************************************************
 * Function: dtrmn_target_loc_img
 * Description: Determine target location and movement patterns in the image.
 ********************************************************************************/
void dtrmn_target_loc_img(void)
{
    // Only consider targets whose bounding boxes are not smashed against the
    // video frame, and at least a certain size (to prevent bad estimation)
    g_tgt_meas_valid = (g_tgt_valid &&
                        (!(g_tgt_center_y_px < target_det_edge_of_frame_buffer ||
                           g_tgt_center_y_px > (g_cam0_img_height_px - target_det_edge_of_frame_buffer)) &&
                         !(g_tgt_center_x_px < target_det_edge_of_frame_buffer ||
                           g_tgt_center_x_px > (g_cam0_img_width_px - target_det_edge_of_frame_buffer)) &&
                         !((g_tgt_width_pix * g_tgt_height_pix) < min_target_bbox_area)));

    if (g_tgt_meas_valid)
    {
        g_target_lost_dbc_ms = (std::chrono::milliseconds)0;
        g_tgt_lost_dbc_sec = (float)0.0;
    }
    else if (target_data_useful_prv && !g_tgt_meas_valid)
    {
        target_data_useful_dbc.start(); // start timing the outage
    }
    else
    {
#if defined(BLD_WSL)

        g_target_lost_dbc_ms += std::chrono::milliseconds((int)(g_app_dt * 1000));

#else

        g_target_lost_dbc_ms = target_data_useful_dbc.getDur(); // how long without a measurement we could use

#endif

        g_tgt_lost_dbc_sec += g_app_dt;
    }

#if defined(BLD_WSL)

    g_tgt_lost = g_tgt_lost_dbc_sec > target_lost_dbc_reset_sec;

#else

    g_tgt_lost = g_target_lost_dbc_ms > target_lost_dbc_reset_ms;

#endif

    if (!g_tgt_lost)
    {
        // Moving average for more smoothing - modifies the history vector
        g_tgt_cntr_offset_x_pix_filt = moving_average(target_x_pix_hist4avg, g_tgt_cntr_offset_y_pix, x_buffer_idx4avg, x_sum);
        g_tgt_cntr_offset_y_pix_filt = moving_average(target_y_pix_hist4avg, g_tgt_cntr_offset_x_pix, y_buffer_idx4avg, y_sum);

        // Convert the offset to meters
        g_tgt_cntr_offset_x_m = g_tgt_cntr_offset_x_pix_filt * g_cam0_m_per_pix;
    }
    else
    {
        g_tgt_cntr_offset_x_pix_filt = (float)0.0;
        g_tgt_cntr_offset_y_pix_filt = (float)0.0;
        g_tgt_cntr_offset_x_m = (float)0.0;
    }

    target_data_useful_prv = g_tgt_meas_valid;
}

/********************************************************************************
 * Function: dtrmn_target_loc_real
 * Description: Calculate the location of the target relative to the drone.
 *              First implementation will be relative to the camera, needs to use
 *              the center mass of the drone in the end though.
 ********************************************************************************/
void dtrmn_target_loc_real(void)
{
    float d_idx_h;
    float d_idx_w;
    float y_idx_d;
    float y_idx_pix;
    float delta_idx_d;
    float delta_idx_pix;
    float delta_d;
    float target_bounding_box_rate;

    if (g_tgt_meas_valid && !g_tgt_lost)
    {
        /* Calculate distance from camera offset */
        g_tgt_los_dist_from_pix_height = powf(g_tgt_height_pix / pix_height_x_coef, (float)1.0 / pix_height_x_pow);
        g_tgt_los_dist_from_pix_width = powf(g_tgt_width_pix / pix_width_x, (float)1.0 / pix_width_x_pow);

        if (g_tgt_aspect_ratio > 0.4f)
        {
            g_tgt_los_dist_meas = g_tgt_los_dist_from_pix_width;
        }
        else if (g_tgt_los_dist_from_pix_height > 2.99f)
        {
            g_tgt_los_dist_meas = g_tgt_los_dist_from_pix_height;
        }
        else
        {
            g_tgt_los_dist_meas = g_tgt_los_dist_from_pix_width;
        }

        /* Calculate true camera angle relative to the ground, adjusting for pitch. */
        g_cam0_comp_angle_rad = (M_PI / (float)2.0) - g_cam0_tilt_rad;
        g_cam0_angle_rad = g_mav_veh_pitch_rad - g_cam0_tilt_rad;

        /* Calculate the x and z distances of the target relative to the drone camera (positive z is down).*/
        // Always compute using a clamped tilt angle
        // requires: #include <algorithm> (for std::clamp)
        // g_cam0_comp_angle_rad = (pi/2) - g_cam0_tilt_rad (already computed)
        float theta = std::clamp(g_cam0_angle_rad, -g_cam0_comp_angle_rad, 0.0f);
        float thabs = std::fabs(theta);

        g_tgt_pos_x_meas = g_tgt_los_dist_meas * cosf(thabs);
        g_tgt_pos_z_meas = g_tgt_los_dist_meas * sinf(thabs);

        /* Calculate shift in target location in the x and z directions based on camera tilt
        angle and drone pitch. When the drone pitch is equal to the camera tilt angle,
        then there is no forward or backward adjustment of the target's position because
        the camera will be parallel to the ground.  This means there can only be an
        adjustment of the target position estimate in the z direction. Make adjustments when
        drone pitch is less than or equal to the camera tilt angle (brings the camera parallel
        to the ground) and greater than a calibratable value. */
        g_cam0_delta_angle_rad = g_cam0_comp_angle_rad + g_mav_veh_pitch_rad;

        if (g_cam0_delta_angle_rad > 0.0f && g_cam0_delta_angle_rad < g_cam0_tilt_rad)
        {
            delta_idx_d = get_float_index(g_tgt_los_dist_meas, &delta_offset_d[0], MAX_IDX_DELTA_D_OFFSET, true);
            delta_idx_pix = get_float_index(std::fabs(g_tgt_cntr_offset_y_pix), &delta_offset_pixels[0], MAX_IDX_DELTA_PIXEL_OFFSET, true);
            delta_d = get_2d_interpolated_value(&delta_offset[0][0], MAX_IDX_DELTA_PIXEL_OFFSET, MAX_IDX_DELTA_D_OFFSET, delta_idx_pix, delta_idx_d);

            g_tgt_pos_x_delta = delta_d * cosf(g_cam0_delta_angle_rad);
            g_tgt_pos_z_delta = delta_d * sinf(g_cam0_delta_angle_rad);

            /* If object center is below the center of the frame */
            if (g_tgt_cntr_offset_y_pix > g_cam0_img_height_cy)
            {
                g_tgt_pos_x_delta = -g_tgt_pos_x_delta;
                g_tgt_pos_z_delta = -g_tgt_pos_z_delta;
            }
        }

        /* Adjust x and z distances based on camera tilt angle */
        g_tgt_pos_x_meas = g_tgt_pos_x_meas + g_tgt_pos_x_delta;
        g_tgt_pos_z_meas = g_tgt_pos_z_meas + g_tgt_pos_z_delta;

        /* Calculate target y offset from the center line of view of the camera based on calibrated forward distance and the
        offset of the center of the target to the side of the center of the video frame in pixels */
        g_tgt_pos_y_meas = g_cam0_m_per_pix * g_tgt_cntr_offset_x_pix;

        if (g_tgt_cntr_offset_x_pix < 0.0f)
        {
            g_tgt_pos_y_meas = -g_tgt_pos_y_meas;
        }

        x_target_prv = g_tgt_pos_x_meas;
        y_target_prv = g_tgt_pos_y_meas;
    }
    else
    {
        g_tgt_pos_x_meas = (float)0.0;
        g_tgt_pos_y_meas = (float)0.0;
        g_tgt_pos_z_meas = (float)0.0;
        g_tgt_los_dist_meas = (float)0.0;
        g_tgt_los_dist_from_pix_height = (float)0.0;
        g_tgt_los_dist_from_pix_width = (float)0.0;
        g_tgt_pos_x_delta = (float)0.0;
        g_tgt_pos_z_delta = (float)0.0;
    }

    loc_target_valid_prv = g_tgt_valid;
}

/********************************************************************************
 * Function: update_target_loc_real
 * Description: Update step in kalman filter.
 ********************************************************************************/
void update_target_loc_real(void)
{
    if (g_tgt_meas_valid && !g_tgt_lost)
    {
        if (target_is_lost_prv)
        {
            arma::colvec x0(n_states, arma::fill::zeros);
            x0(0) = g_tgt_pos_x_meas; // pos x
            x0(1) = g_tgt_pos_y_meas; // pos y
            // vx, vy, ax, ay left at 0

            kf_loc.InitSystemState(x0);
            kf_loc_reset = true;
        }

        // Measurement vector (observations)
        colvec z(2);
        z << g_tgt_pos_x_meas << g_tgt_pos_y_meas;

        // Define the system transition matrix A_loc (discretized)
        // TODO: update the state transition matrix A_loc
        // Ensure g_app_dt is valid (std::isnan(g_app_dt) || std::isinf(g_app_dt) || g_app_dt <= 0)

        // No external control input for now
        colvec u(1, fill::zeros);

        // Perform Kalman filter update
        kf_loc.Kalmanf(z, u);

        // Retrieve the updated state
        colvec *state = kf_loc.GetCurrentEstimatedState();
        g_tgt_pos_x_est = state->at(0);
        g_tgt_pos_y_est = state->at(1);
        g_tgt_vel_x_est = state->at(2);
        g_tgt_vel_y_est = state->at(3);
        g_tgt_acc_x_est = state->at(4);
        g_tgt_acc_y_est = state->at(5);
    }

    if (g_tgt_lost)
    {
        g_tgt_pos_x_est = (float)0.0;
        g_tgt_pos_y_est = (float)0.0;
        g_tgt_vel_x_est = (float)0.0;
        g_tgt_vel_y_est = (float)0.0;
        g_tgt_acc_x_est = (float)0.0;
        g_tgt_acc_y_est = (float)0.0;
    }

    target_is_lost_prv = g_tgt_lost;
};

/********************************************************************************
 * Function: ~TargetLocalization
 * Description: TargetLocalization class destructor.
 ********************************************************************************/
TargetLocalization::~TargetLocalization(void) {};

/********************************************************************************
 * Function: init
 * Description: Initialize all TargetLocalization target variables.  Run once at the start
 *              of the program.
 ********************************************************************************/
bool TargetLocalization::init(void)
{
    get_localization_params();
    init_kf_loc();

    g_tgt_los_dist_from_pix_height = (float)0.0;
    g_tgt_los_dist_from_pix_width = (float)0.0;
    g_tgt_pos_x_meas = (float)0.0;
    g_tgt_pos_y_meas = (float)0.0;
    g_tgt_pos_z_meas = (float)0.0;
    g_tgt_los_dist_meas = (float)0.0;
    g_cam0_delta_angle_rad = (float)0.0;
    g_cam0_angle_rad = (float)0.0;
    g_tgt_pos_x_delta = (float)0.0;
    g_tgt_pos_z_delta = (float)0.0;
    g_cam0_comp_angle_rad = (float)0.0;
    x_target_prv = (float)0.0;
    y_target_prv = (float)0.0;
    loc_target_valid_prv = false;
    g_tgt_meas_valid = false;
    camera_comp_angle_abs = (float)0.0;
    g_cam0_fov_height = (float)0.0;
    g_cam0_m_per_pix = (float)0.00001;
    g_tgt_cntr_offset_x_m = (float)0.0;
    target_y_pix_hist4avg.assign(y_pix_window, (float)0.0);
    target_x_pix_hist4avg.assign(x_pix_window, (float)0.0);
    g_tgt_cntr_offset_x_pix_filt = (float)0.0;
    g_tgt_cntr_offset_y_pix_filt = (float)0.0;
    y_buffer_idx4avg = (int)0;
    y_sum = (float)0.0;
    x_buffer_idx4avg = (int)0;
    x_sum = (float)0.0;
    g_cam0_los_m = (float)0.0;
    target_valid_prv = false;
    kf_loc_reset = false;
    g_tgt_lost = false;
    target_is_lost_prv = false;
    target_data_useful_prv = false;

    return true;
}

/********************************************************************************
 * Function: loop
 * Description: Return control parameters for the vehicle to TargetLocalization a designated
 *              target at a distance.
 ********************************************************************************/
void TargetLocalization::loop(void)
{
    calc_fov();
    dtrmn_target_loc_img();
    dtrmn_target_loc_real();
    update_target_loc_real();
}

/********************************************************************************
 * Function: shutdown
 * Description: Clean up code to run before program exits.
 ********************************************************************************/
void TargetLocalization::shutdown(void)
{
    // place clean up code here
}

#endif // ENABLE_CV
