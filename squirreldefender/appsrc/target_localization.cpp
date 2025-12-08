#ifdef ENABLE_CV

/********************************************************************************
 * @file    target_localization.cpp
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Calculate the x, y, z offset of detected targets relative to the
 *          drone in engineering units.  This module filters out unusable
 *          bounding box measurements, converts usable measurements to an x, y,
 *          z, distance.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include <cmath>
#include <algorithm>

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

// Target measurement state - raw and derived measurements from bounding boxes and vehicle attitude
float g_tgt_los_dist_from_pix_height; // LOS distance estimate from bbox height
float g_tgt_los_dist_from_pix_width;  // LOS distance estimate from bbox width
float g_tgt_los_dist_meas;            // Final chosen LOS measurement

float g_tgt_pos_x_meas; // Measured forward distance (m)
float g_tgt_pos_y_meas; // Measured sideways distance (m)
float g_tgt_pos_z_meas; // Measured vertical/drop distance (m)

bool g_tgt_meas_valid = false; // Validity of current measurement
bool g_tgt_lost = false;       // Target loss flag

// Camera geometry and angles (runtime)
float g_cam0_angle_rad;       // Camera angle relative to ground
float g_cam0_comp_angle_rad;  // π/2 - tilt
float g_cam0_delta_angle_rad; // Camera tilt + pitch
float g_cam0_m_per_pix;       // Meters per pixel at current geometry
float g_cam0_fov_height;      // Scene height in meters
float g_cam0_los_m;           // Camera line-of-sight distance

// Localization correction from tilt and pixel offsets
float g_tgt_pos_x_delta; // X correction (tilt induced)
float g_tgt_pos_z_delta; // Z correction (tilt induced)

float g_tgt_cntr_offset_x_pix_filt; // Filtered horizontal pixel offset
float g_tgt_cntr_offset_y_pix_filt; // Filtered vertical pixel offset
float g_tgt_cntr_offset_x_m;        // Horizontal offset converted to meters

// Filtering buffers (runtime)
std::vector<float> tgt_cntr_offset_y_pix_hist;
std::vector<float> tgt_cntr_offset_x_pix_hist;

int tgt_cntr_offset_x_pix_hist_idx; // Circular buffer index
int tgt_cntr_offset_y_pix_hist_idx;

float tgt_cntr_offset_x_pix_hist_sum; // Running sums for moving average
float tgt_cntr_offset_y_pix_hist_sum;

// EKF runtime objects
KF kf_tgt_loc;

arma::mat kf_tgt_loc_A; // Transition matrix
arma::mat kf_tgt_loc_B; // Control matrix
arma::mat kf_tgt_loc_H; // Observation matrix
arma::mat kf_tgt_loc_Q; // Process noise covariance
arma::mat kf_tgt_loc_R; // Measurement noise covariance
arma::mat kf_tgt_loc_P; // Error covariance

arma::colvec kf_tgt_loc_x0; // Initial EKF state
arma::colvec kf_tgt_loc_u0; // Control vector (unused)

// EKF output estimates (runtime)
float g_tgt_pos_x_est;
float g_tgt_pos_y_est;
float g_tgt_vel_x_est;
float g_tgt_vel_y_est;
float g_tgt_acc_x_est;
float g_tgt_acc_y_est;

// Target loss timers (runtime)
Timer tgt_lost_dbc;
std::chrono::milliseconds g_target_lost_dbc_ms;

float g_tgt_lost_dbc_sec;
bool tgt_lost_prv;
bool tgt_meas_valid_prv;

// Previous-frame values
float tgt_pos_x_meas_prv;
float tgt_pos_y_meas_prv;

bool tgt_valid_prv;

float tgt_pos_x_est_prv;
float tgt_pos_y_est_prv;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

// Lookup tables for tilt-dependent forward/backward correction
const float delta_offset_d[MAX_IDX_DELTA_D_OFFSET] = {
    (float)0.0, (float)1.0, (float)2.0, (float)3.0,
    (float)4.0, (float)5.0, (float)6.0, (float)7.0};

const float delta_offset_pixels[MAX_IDX_DELTA_PIXEL_OFFSET] = {
    (float)0.0, (float)50.0, (float)100.0, (float)150.0,
    (float)200.0, (float)250.0, (float)300.0, (float)350.0};

const float delta_offset[MAX_IDX_DELTA_PIXEL_OFFSET][MAX_IDX_DELTA_D_OFFSET] = {
    {(float)0.0, (float)0.0, (float)0.0, (float)0.0, (float)0.0, (float)0.0, (float)0.0, (float)0.0},
    {(float)0.0, (float)0.08, (float)0.11, (float)0.15, (float)0.20, (float)0.27, (float)0.30, (float)0.40},
    {(float)0.0, (float)0.17, (float)0.23, (float)0.30, (float)0.40, (float)0.53, (float)0.60, (float)0.80},
    {(float)0.0, (float)0.25, (float)0.34, (float)0.45, (float)0.60, (float)0.80, (float)0.90, (float)1.20},
    {(float)0.0, (float)0.34, (float)0.45, (float)0.60, (float)0.80, (float)1.06, (float)1.20, (float)1.60},
    {(float)0.0, (float)0.42, (float)0.56, (float)0.75, (float)1.00, (float)1.33, (float)1.50, (float)2.00},
    {(float)0.0, (float)0.51, (float)0.68, (float)0.90, (float)1.20, (float)1.60, (float)1.80, (float)2.39},
    {(float)0.0, (float)0.59, (float)0.79, (float)1.05, (float)1.40, (float)1.86, (float)2.10, (float)2.79}};

// EKF calibration
const float kf_tgt_loc_dt = (float)0.05;

float kf_tgt_loc_H_00 = (float)1.0;
float kf_tgt_loc_H_11 = (float)1.0;

float kf_tgt_loc_Q_00 = (float)0.01;
float kf_tgt_loc_Q_11 = (float)0.0001;
float kf_tgt_loc_Q_22 = (float)0.00001;
float kf_tgt_loc_Q_33 = (float)0.00001;
float kf_tgt_loc_Q_44 = (float)0.00001;
float kf_tgt_loc_Q_55 = (float)0.00001;

float kf_tgt_loc_R_00 = (float)10.0;
float kf_tgt_loc_R_11 = (float)40.0;

float kf_tgt_loc_P_00 = (float)0.1;
float kf_tgt_loc_P_11 = (float)0.1;
float kf_tgt_loc_P_22 = (float)0.1;
float kf_tgt_loc_P_33 = (float)0.1;
float kf_tgt_loc_P_44 = (float)0.1;
float kf_tgt_loc_P_55 = (float)0.1;

const int kf_tgt_loc_n_states = (int)6;
const int kf_tgt_loc_n_meas = (int)2;

// LOS estimation calibration
float tgt_los_known_height_m = (float)1.778;

float tgt_los_pix_height_coef = (float)1680.557;
float tgt_los_pix_height_pow = (float)-0.863;

float tgt_los_pix_width_coef = (float)540.456;
float tgt_los_pix_width_pow = (float)-0.758;

// Target validation calibration
float target_det_edge_of_frame_buffer = (float)50.0;
float min_target_bbox_area = (float)3500.0;

// Pixel-centroid smoothing window sizes (JSON-loaded)
int tgt_cntr_offset_x_pix_hist_size = (int)1;
int tgt_cntr_offset_y_pix_hist_size = (int)1;

// Target lost debounce (JSON-loaded)
float target_lost_dbc_reset_sec = (float)1.0;
std::chrono::milliseconds target_lost_dbc_reset_ms = std::chrono::milliseconds((int)1000);

/********************************************************************************
 * Function definitions
 ********************************************************************************/
void get_localization_params(void);
void init_kf_target_loc(void);
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

    // kf_tgt_loc_H (Observation matrix, 2x6)
    kf_tgt_loc_H_00 = localization_params.get_float_param("target_loc_params.h_pos_x_00");
    kf_tgt_loc_H_11 = localization_params.get_float_param("target_loc_params.h_pos_y_11");

    // kf_tgt_loc_Q (Process noise covariance diagonals, 6x6)
    kf_tgt_loc_Q_00 = localization_params.get_float_param("target_loc_params.q_pos_x_00");
    kf_tgt_loc_Q_11 = localization_params.get_float_param("target_loc_params.q_pos_y_11");
    kf_tgt_loc_Q_22 = localization_params.get_float_param("target_loc_params.q_vel_x_22");
    kf_tgt_loc_Q_33 = localization_params.get_float_param("target_loc_params.q_vel_y_33");
    kf_tgt_loc_Q_44 = localization_params.get_float_param("target_loc_params.q_acc_x_44");
    kf_tgt_loc_Q_55 = localization_params.get_float_param("target_loc_params.q_acc_y_55");

    // kf_tgt_loc_R (Measurement noise covariance diagonals, 2x2)
    kf_tgt_loc_R_00 = localization_params.get_float_param("target_loc_params.r_pos_x_00");
    kf_tgt_loc_R_11 = localization_params.get_float_param("target_loc_params.r_pos_y_11");

    // kf_tgt_loc_P (Estimate covariance diagonals, 6x6)
    kf_tgt_loc_P_00 = localization_params.get_float_param("target_loc_params.p_pos_x_00");
    kf_tgt_loc_P_11 = localization_params.get_float_param("target_loc_params.p_pos_y_11");
    kf_tgt_loc_P_22 = localization_params.get_float_param("target_loc_params.p_vel_x_22");
    kf_tgt_loc_P_33 = localization_params.get_float_param("target_loc_params.p_vel_y_33");
    kf_tgt_loc_P_44 = localization_params.get_float_param("target_loc_params.p_acc_x_44");
    kf_tgt_loc_P_55 = localization_params.get_float_param("target_loc_params.p_acc_y_55");

    // Target centroid moving average and curve fit window sizes
    tgt_cntr_offset_x_pix_hist_size = localization_params.get_int_param("target_loc_params.target_bbox_center_x_history_size");
    tgt_cntr_offset_y_pix_hist_size = localization_params.get_int_param("target_loc_params.target_bbox_center_y_history_size");

    tgt_los_pix_height_coef = localization_params.get_float_param("target_loc_params.target_los_height_eq_coef");
    tgt_los_pix_height_pow = localization_params.get_float_param("target_loc_params.target_los_height_eq_pow");
    tgt_los_pix_width_coef = localization_params.get_float_param("target_loc_params.target_los_width_eq_coef");
    tgt_los_pix_width_pow = localization_params.get_float_param("target_loc_params.target_los_width_eq_pow");
    tgt_los_known_height_m = localization_params.get_float_param("target_loc_params.known_object_height_all_dist");
    target_det_edge_of_frame_buffer = localization_params.get_float_param("target_loc_params.target_bbox_min_dist_from_edge");
    min_target_bbox_area = localization_params.get_float_param("target_loc_params.target_bbox_min_area");
    int target_lost_dbc_reset_val = localization_params.get_float_param("target_loc_params.target_bbox_lost_min_time_ms");
    target_lost_dbc_reset_sec = (float)target_lost_dbc_reset_val / (float)1000.0;
    target_lost_dbc_reset_ms = static_cast<std::chrono::milliseconds>(target_lost_dbc_reset_val);
}

/********************************************************************************
 * Function: init_kf_target_loc
 * Description: Initialize kalman filter for target location estimation.
 ********************************************************************************/
void init_kf_target_loc(void)
{
    tgt_pos_x_est_prv = 0.0;
    tgt_pos_y_est_prv = 0.0;
    g_tgt_pos_x_est = 0.0;
    g_tgt_pos_y_est = 0.0;
    g_tgt_vel_x_est = 0.0;
    g_tgt_vel_y_est = 0.0;
    g_tgt_acc_x_est = 0.0;
    g_tgt_acc_y_est = 0.0;

    // Resize matrices and set to zero initially
    kf_tgt_loc_A.set_size(kf_tgt_loc_n_states, kf_tgt_loc_n_states);
    kf_tgt_loc_A.zeros();
    kf_tgt_loc_B.set_size(kf_tgt_loc_n_states, 1);
    kf_tgt_loc_B.zeros();
    kf_tgt_loc_H.set_size(kf_tgt_loc_n_meas, kf_tgt_loc_n_states);
    kf_tgt_loc_H.zeros();
    kf_tgt_loc_Q.set_size(kf_tgt_loc_n_states, kf_tgt_loc_n_states);
    kf_tgt_loc_Q.zeros();
    kf_tgt_loc_R.set_size(kf_tgt_loc_n_meas, kf_tgt_loc_n_meas);
    kf_tgt_loc_R.zeros();
    kf_tgt_loc_P.set_size(kf_tgt_loc_n_states, kf_tgt_loc_n_states);
    kf_tgt_loc_P.zeros();

    // Define system dynamics (State transition matrix kf_tgt_loc_A)
    kf_tgt_loc_A << 1 << 0 << kf_tgt_loc_dt << 0 << 0.5 * pow(kf_tgt_loc_dt, 2) << 0 << arma::endr
                 << 0 << 1 << 0 << kf_tgt_loc_dt << 0 << 0.5 * pow(kf_tgt_loc_dt, 2) << arma::endr
                 << 0 << 0 << 1 << 0 << kf_tgt_loc_dt << 0 << arma::endr
                 << 0 << 0 << 0 << 1 << 0 << kf_tgt_loc_dt << arma::endr
                 << 0 << 0 << 0 << 0 << 1 << 0 << arma::endr
                 << 0 << 0 << 0 << 0 << 0 << 1 << arma::endr;

    // Observation matrix (Only x and y are observed)
    kf_tgt_loc_H = arma::zeros<arma::mat>(2, 6);
    kf_tgt_loc_H(0, 0) = kf_tgt_loc_H_00;
    kf_tgt_loc_H(1, 1) = kf_tgt_loc_H_11;

    // Model covariance matrix kf_tgt_loc_Q (process noise)
    kf_tgt_loc_Q = arma::zeros<arma::mat>(6, 6);
    kf_tgt_loc_Q(0, 0) = kf_tgt_loc_Q_00;
    kf_tgt_loc_Q(1, 1) = kf_tgt_loc_Q_11;
    kf_tgt_loc_Q(2, 2) = kf_tgt_loc_Q_22;
    kf_tgt_loc_Q(3, 3) = kf_tgt_loc_Q_33;
    kf_tgt_loc_Q(4, 4) = kf_tgt_loc_Q_44;
    kf_tgt_loc_Q(5, 5) = kf_tgt_loc_Q_55;

    // Measurement covariance matrix kf_tgt_loc_R (sensor noise)
    kf_tgt_loc_R = arma::zeros<arma::mat>(2, 2);
    kf_tgt_loc_R(0, 0) = kf_tgt_loc_R_00;
    kf_tgt_loc_R(1, 1) = kf_tgt_loc_R_11;

    // Estimate covariance matrix kf_tgt_loc_P (initial trust in the estimate)
    kf_tgt_loc_P = arma::zeros<arma::mat>(6, 6);
    kf_tgt_loc_P(0, 0) = kf_tgt_loc_P_00;
    kf_tgt_loc_P(1, 1) = kf_tgt_loc_P_11;
    kf_tgt_loc_P(2, 2) = kf_tgt_loc_P_22;
    kf_tgt_loc_P(3, 3) = kf_tgt_loc_P_33;
    kf_tgt_loc_P(4, 4) = kf_tgt_loc_P_44;
    kf_tgt_loc_P(5, 5) = kf_tgt_loc_P_55;

    // Initialize Kalman Filter
    kf_tgt_loc.InitSystem(kf_tgt_loc_A, kf_tgt_loc_B, kf_tgt_loc_H, kf_tgt_loc_Q, kf_tgt_loc_R);

    // Set initial state (assuming stationary at origin)
    kf_tgt_loc_x0.set_size(kf_tgt_loc_n_states);
    kf_tgt_loc_x0.zeros(); // Start with zero velocity and acceleration
    kf_tgt_loc.InitSystemState(kf_tgt_loc_x0);

    // Resize and reset kf_tgt_loc_u0
    kf_tgt_loc_u0.set_size(kf_tgt_loc_n_meas);
    kf_tgt_loc_u0.zeros();
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
        ghost_object_height = tgt_los_pix_height_coef * powf(g_cam0_los_m, tgt_los_pix_height_pow);
    }
    // How many meters a single pixel represents at the line of sight distance
    // TODO: When to use actual vs ghost object height?
    // Uses "ghost object" height - the height of the known object at the LOS distance
    // g_cam0_m_per_pix = tgt_los_known_height_m / ghost_object_height;
    // Currently gonna use the target actual height
    // Only compute m_per_pixel if we have a valid bbox height
    if (g_tgt_height_pix > (float)0.0f)
    {
        g_cam0_m_per_pix = tgt_los_known_height_m / g_tgt_height_pix;
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
    else if (tgt_meas_valid_prv && !g_tgt_meas_valid)
    {
        tgt_lost_dbc.start(); // start timing the outage
    }
    else
    {
#if defined(BLD_WSL)

        g_target_lost_dbc_ms += std::chrono::milliseconds((int)(g_app_dt * 1000));

#else

        g_target_lost_dbc_ms = tgt_lost_dbc.getDur(); // how long without a measurement we could use

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
        g_tgt_cntr_offset_x_pix_filt = moving_average(tgt_cntr_offset_x_pix_hist, g_tgt_cntr_offset_x_pix, tgt_cntr_offset_x_pix_hist_idx, tgt_cntr_offset_x_pix_hist_sum);
        g_tgt_cntr_offset_y_pix_filt = moving_average(tgt_cntr_offset_y_pix_hist, g_tgt_cntr_offset_y_pix, tgt_cntr_offset_y_pix_hist_idx, tgt_cntr_offset_y_pix_hist_sum);

        // Convert the offset to meters
        g_tgt_cntr_offset_x_m = g_tgt_cntr_offset_x_pix_filt * g_cam0_m_per_pix;
    }
    else
    {
        g_tgt_cntr_offset_x_pix_filt = (float)0.0;
        g_tgt_cntr_offset_y_pix_filt = (float)0.0;
        g_tgt_cntr_offset_x_m = (float)0.0;
    }

    tgt_meas_valid_prv = g_tgt_meas_valid;
}

/********************************************************************************
 * Function: dtrmn_target_loc_real
 * Description:
 *      Calculate the location of the target relative to the drone.
 *      First implementation will be relative to the camera, needs to use
 *      the center mass of the drone in the end though.
 *
 *      x is the forward distance from the drone - positive is in front of drone
 *      y is the left to right distance from center- positive is to the right of
 *          the drone
 *      z is the veritcal distance below the drone
 *
 *      Steps:
 *          1. Estimate LOS distance using model relating bbox size to distance.
 *          2. Correct the estimate based on the camera's tilt angle (account for
 *             drone pitch)
 *          3. Apply corrections to LOS distance estimates
 *          4. Convert pixel offset to y distance in meters
 *
 *      Larger bbox height/width means target is closer to the drone.
 *      Cameras don't measure distance inherently - it is inferred from geometry
 *
 *      Why tilt matters:
 *          When the drone pitches forward/back, the camera sees the same object
 *          in a different part of the image, which changes the inferred
 *          distance.
 ********************************************************************************/
void dtrmn_target_loc_real(void)
{
    float delta_idx_d;
    float delta_idx_pix;
    float delta_d;

    g_tgt_pos_x_delta = (float)0.0;
    g_tgt_pos_z_delta = (float)0.0;

    if (g_tgt_meas_valid && !g_tgt_lost)
    {
        /* Calculate distance from camera offset */
        g_tgt_los_dist_from_pix_height = powf(g_tgt_height_pix / tgt_los_pix_height_coef, (float)1.0 / tgt_los_pix_height_pow);
        g_tgt_los_dist_from_pix_width = powf(g_tgt_width_pix / tgt_los_pix_width_coef, (float)1.0 / tgt_los_pix_width_pow);

        if (g_tgt_aspect_ratio > (float)0.4)
        {
            g_tgt_los_dist_meas = g_tgt_los_dist_from_pix_width;
        }
        else if (g_tgt_los_dist_from_pix_height > (float)2.99)
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

        if (g_tgt_cntr_offset_x_pix < (float)0.0)
        {
            g_tgt_pos_y_meas = -g_tgt_pos_y_meas;
        }

        tgt_pos_x_meas_prv = g_tgt_pos_x_meas;
        tgt_pos_y_meas_prv = g_tgt_pos_y_meas;
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

    tgt_valid_prv = g_tgt_valid;
}

/********************************************************************************
 * Function: update_target_loc_real
 * Description:
 *      Smooth out the target position estimate by fusing bbox target distance
 *      estimate with model based prediction.
 *
 *      Target assumed to move with constant acceleration kinematics model. This
 *      removes jitter in the target x/y position estimates.
 ********************************************************************************/
void update_target_loc_real(void)
{
    if (g_tgt_meas_valid && !g_tgt_lost)
    {
        if (tgt_lost_prv)
        {
            arma::colvec x0(kf_tgt_loc_n_states, arma::fill::zeros);
            x0(0) = g_tgt_pos_x_meas; // pos x
            x0(1) = g_tgt_pos_y_meas; // pos y
            // vx, vy, ax, ay left at 0

            kf_tgt_loc.InitSystemState(x0);
        }

        // Measurement vector (observations)
        arma::colvec z(2);
        z << g_tgt_pos_x_meas << g_tgt_pos_y_meas;

        // Define the system transition matrix kf_tgt_loc_A (discretized)
        // TODO: update the state transition matrix kf_tgt_loc_A
        // Ensure g_app_dt is valid (std::isnan(g_app_dt) || std::isinf(g_app_dt) || g_app_dt <= 0)

        // No external control input for now
        arma::colvec u(1, arma::fill::zeros);

        // Perform Kalman filter update
        kf_tgt_loc.Kalmanf(z, u);

        // Retrieve the updated state
        arma::colvec *state = kf_tgt_loc.GetCurrentEstimatedState();
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

    tgt_lost_prv = g_tgt_lost;
};

/********************************************************************************
 * Function: ~TargetLocalization
 * Description: TargetLocalization class destructor.
 ********************************************************************************/
TargetLocalization::~TargetLocalization(void) {};

/********************************************************************************
 * Function: init
 * Description: Initialize localization variables
 ********************************************************************************/
bool TargetLocalization::init(void)
{
    get_localization_params();
    init_kf_target_loc();

    g_cam0_m_per_pix = (float)0.00001;
    tgt_cntr_offset_y_pix_hist.assign(tgt_cntr_offset_y_pix_hist_size, (float)0.0);
    tgt_cntr_offset_x_pix_hist.assign(tgt_cntr_offset_x_pix_hist_size, (float)0.0);

    return true;
}

/********************************************************************************
 * Function: loop
 * Description: Process tracked/detected targets and locate them in engineering
 *              units.
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
