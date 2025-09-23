#ifdef ENABLE_CV

/********************************************************************************
 * @file    localize_target.cpp
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Localize the x, y, z offset of the target relative to the vehicle.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "localize_target.h"

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
float g_d_target_h;
float g_d_target_w;
float g_x_target;
float g_y_target;
float g_z_target;
float g_d_target;
float g_delta_angle;       //
float g_camera_tilt_angle; // Angle of the camera relative to the ground, compensating for pitch
float g_delta_d_x;
float g_delta_d_z;
float g_camera_comp_angle;
float x_target_prv;
float y_target_prv;
bool y_target_hyst_actv;
float y_target_latched;
bool loc_target_valid_prv;
bool g_target_data_useful;
float loc_target_center_x_prv;
float loc_target_center_y_prv;
float g_x_target_ekf;
float g_y_target_ekf;
float g_vx_target_ekf;
float g_vy_target_ekf;
float g_ax_target_ekf;
float g_ay_target_ekf;
float x_target_ekf_prv;
float y_target_ekf_prv;
float camera_comp_angle_abs;
float g_fov_height;
float g_meter_per_pix;
float g_target_cntr_offset_x_m;
float g_line_of_sight;

// Buffers for moving average of target centroid
std::vector<float> target_y_pix_hist4avg;
std::vector<float> target_x_pix_hist4avg;
int x_pix_window;
int y_pix_window;
float g_target_cntr_offset_x_mov_avg;
float g_target_cntr_offset_y_mov_avg;
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

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/
const float center_of_frame_width = 640.0f;
const float center_of_frame_height = 360.0f;
const float camera_fixed_angle = 0.436f; // radians
const int n_states = 6;
const int n_meas = 2;
const float KNOWN_OBJECT_HEIGHT = (float)1.778;

const float d_offset_w[MAX_IDX_D_WIDTH] = {
    0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5, 8.0, 8.5, 9.0, 9.5,
    10.0, 10.5, 11.0, 11.5, 12.0, 12.5, 13.0, 13.5, 14.0, 14.5, 15.0};

const float width_index[MAX_IDX_D_WIDTH] = {
    1280.000, 913.989, 540.456, 397.451, 319.580, 269.850, 235.019, 209.101, 188.973, 172.832, 159.566, 148.445, 138.970, 130.789, 123.645,
    117.345, 111.742, 106.724, 102.198, 98.095, 94.354, 90.928, 87.778, 84.869, 82.175, 79.671, 77.338, 75.157, 73.113, 71.194, 69.388};

const float d_offset_h[MAX_IDX_D_HEIGHT] = {
    3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5, 8.0, 8.5, 9.0, 9.5,
    10.0, 10.5, 11.0, 11.5, 12.0, 12.5, 13.0, 13.5, 14.0, 14.5, 15.0};

const float height_index[MAX_IDX_D_HEIGHT] = {
    651.175, 570.063, 508.014, 458.914, 419.027, 385.940, 358.021, 334.125, 313.425, 295.308, 279.310, 265.073, 252.314,
    240.812, 230.384, 220.885, 212.193, 204.207, 196.843, 190.029, 183.705, 177.818, 172.324, 167.183, 162.363};

//
const float y_offset_d[MAX_IDX_Y_D_OFFSET] = {
    0.00f, 1.00f, 2.00f, 3.00f, 4.00f, 5.00f, 6.00f, 7.00f, 8.00f, 9.00f,
    10.00f, 11.00f, 12.00f, 13.00f, 14.00f, 15.00f};

const float y_offset_pixels[MAX_IDX_Y_PIXEL_OFFSET] = {
    0.00f, 50.00f, 100.00f, 150.00f, 200.00f, 250.00f, 300.00f, 350.00f,
    400.00f, 450.00f, 500.00f, 550.00f, 600.00f};

// Define the 2D array for target y values
const float y_offset[MAX_IDX_Y_PIXEL_OFFSET][MAX_IDX_Y_D_OFFSET] = {
    {0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f},
    {0.00f, 0.08f, 0.11f, 0.15f, 0.20f, 0.27f, 0.30f, 0.40f, 0.50f, 0.67f, 0.88f, 1.18f, 1.56f, 2.08f, 2.77f, 3.68f},
    {0.00f, 0.17f, 0.23f, 0.30f, 0.40f, 0.53f, 0.60f, 0.80f, 1.00f, 1.33f, 1.77f, 2.35f, 3.13f, 4.16f, 5.53f, 7.36f},
    {0.00f, 0.25f, 0.34f, 0.45f, 0.60f, 0.80f, 0.90f, 1.20f, 1.50f, 2.00f, 2.65f, 3.53f, 4.69f, 6.24f, 8.30f, 11.04f},
    {0.00f, 0.34f, 0.45f, 0.60f, 0.80f, 1.06f, 1.20f, 1.60f, 2.00f, 2.66f, 3.54f, 4.71f, 6.26f, 8.32f, 11.07f, 14.72f},
    {0.00f, 0.42f, 0.56f, 0.75f, 1.00f, 1.33f, 1.50f, 2.00f, 2.50f, 3.33f, 4.42f, 5.88f, 7.82f, 10.40f, 13.84f, 18.40f},
    {0.00f, 0.51f, 0.68f, 0.90f, 1.20f, 1.60f, 1.80f, 2.39f, 3.00f, 3.99f, 5.31f, 7.06f, 9.39f, 12.48f, 16.60f, 22.08f},
    {0.00f, 0.59f, 0.79f, 1.05f, 1.40f, 1.86f, 2.10f, 2.79f, 3.50f, 4.66f, 6.19f, 8.23f, 10.95f, 14.57f, 19.37f, 25.76f},
    {0.00f, 0.68f, 0.90f, 1.20f, 1.60f, 2.13f, 2.40f, 3.19f, 4.00f, 5.32f, 7.08f, 9.41f, 12.52f, 16.65f, 22.14f, 29.45f},
    {0.00f, 0.76f, 1.01f, 1.35f, 1.80f, 2.39f, 2.70f, 3.59f, 4.50f, 5.99f, 7.96f, 10.59f, 14.08f, 18.73f, 24.91f, 33.13f},
    {0.00f, 0.84f, 1.13f, 1.50f, 2.00f, 2.66f, 3.00f, 3.99f, 5.00f, 6.65f, 8.84f, 11.76f, 15.65f, 20.81f, 27.67f, 36.81f},
    {0.00f, 0.93f, 1.24f, 1.65f, 2.20f, 2.93f, 3.30f, 4.39f, 5.50f, 7.32f, 9.73f, 12.94f, 17.21f, 22.89f, 30.44f, 40.49f},
    {0.00f, 1.01f, 1.35f, 1.80f, 2.40f, 3.19f, 3.60f, 4.79f, 6.00f, 7.98f, 10.61f, 14.12f, 18.77f, 24.97f, 33.21f, 44.17f}};

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

/********************************************************************************
 * Function definitions
 ********************************************************************************/
void get_localization_params(void);
void init_kf_loc(void);
void dtrmn_target_location(void);
void update_target_location(void);
void calc_vertical_fov(void);
void dtrmn_target_loc_img(void);

/********************************************************************************
 * Function: get_localization_params
 * Description: Calibratable parameters for target localization.
 ********************************************************************************/
void get_localization_params(void)
{
    ParamReader localization_params("../params.json");

    // H_loc (Observation matrix, 2x6)
    H_loc_00 = localization_params.get_float_param("Localization_Params", "H_loc_00");
    H_loc_11 = localization_params.get_float_param("Localization_Params", "H_loc_11");

    // Q_loc (Process noise covariance diagonals, 6x6)
    Q_loc_00 = localization_params.get_float_param("Localization_Params", "Q_loc_00");
    Q_loc_11 = localization_params.get_float_param("Localization_Params", "Q_loc_11");
    Q_loc_22 = localization_params.get_float_param("Localization_Params", "Q_loc_22");
    Q_loc_33 = localization_params.get_float_param("Localization_Params", "Q_loc_33");
    Q_loc_44 = localization_params.get_float_param("Localization_Params", "Q_loc_44");
    Q_loc_55 = localization_params.get_float_param("Localization_Params", "Q_loc_55");

    // R_loc (Measurement noise covariance diagonals, 2x2)
    R_loc_00 = localization_params.get_float_param("Localization_Params", "R_loc_00");
    R_loc_11 = localization_params.get_float_param("Localization_Params", "R_loc_11");

    // P_loc (Estimate covariance diagonals, 6x6)
    P_loc_00 = localization_params.get_float_param("Localization_Params", "P_loc_00");
    P_loc_11 = localization_params.get_float_param("Localization_Params", "P_loc_11");
    P_loc_22 = localization_params.get_float_param("Localization_Params", "P_loc_22");
    P_loc_33 = localization_params.get_float_param("Localization_Params", "P_loc_33");
    P_loc_44 = localization_params.get_float_param("Localization_Params", "P_loc_44");
    P_loc_55 = localization_params.get_float_param("Localization_Params", "P_loc_55");

    // Target centroid moving average and curve fit window sizes
    x_pix_window = localization_params.get_int_param("Localization_Params", "X_pix_mov_avg_window_size");
    y_pix_window = localization_params.get_int_param("Localization_Params", "Y_pix_mov_avg_window_size");
}

/********************************************************************************
 * Function: init_kf_loc
 * Description: Initialize kalman filter for target location estimation.
 ********************************************************************************/
void init_kf_loc(void)
{
    x_target_ekf_prv = 0.0;
    y_target_ekf_prv = 0.0;
    g_x_target_ekf = 0.0;
    g_y_target_ekf = 0.0;
    g_vx_target_ekf = 0.0;
    g_vy_target_ekf = 0.0;
    g_ax_target_ekf = 0.0;
    g_ay_target_ekf = 0.0;

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
 * Function: dtrmn_target_location
 * Description: Calculate the location of the target relative to the drone.
 *              First implementation will be relative to the camera, needs to use
 *              the center mass of the drone in the end though.
 ********************************************************************************/
void dtrmn_target_location(void)
{
    float d_idx_h;
    float d_idx_w;
    float y_idx_d;
    float y_idx_pix;
    float delta_idx_d;
    float delta_idx_pix;
    float delta_d;
    float target_bounding_box_rate;

    // Only consider targets whose bounding boxes are not smashed against the
    // video frame, and at least a certain size (to prevent bad estimation)
    g_target_data_useful = (g_target_valid &&
                            (!(g_target_center_x < 50.0 || g_target_center_x > 670) &&
                             !(g_target_center_y < 50.0 || g_target_center_x > 1230) &&
                             !((g_target_width * g_target_height) < 3500.0)));

    if (g_target_data_useful)
    {
        /* Calculate distance from camera offset */
        const float PIX_HEIGHT_X = (float)1680.557;
        const float PIX_HEIGHT_X_POW = (float)(-0.863);
        const float PIX_WIDTH_X = (float)540.456;
        const float PIX_WIDTH_X_POW = (float)(-0.758);

        g_d_target_h = powf(g_target_height / PIX_HEIGHT_X, (float)1.0 / PIX_HEIGHT_X_POW);
        g_d_target_w = powf(g_target_width / PIX_WIDTH_X, (float)1.0 / PIX_WIDTH_X_POW);

        if (g_target_aspect > 0.4f)
        {
            g_d_target = g_d_target_w;
        }
        else if (g_d_target_h > 2.99f)
        {
            g_d_target = g_d_target_h;
        }
        else
        {
            g_d_target = g_d_target_w;
        }

        /* Calculate true camera angle relative to the ground, adjusting for pitch. */
        g_camera_comp_angle = (PI / (float)2.0) - camera_fixed_angle;
        g_camera_tilt_angle = g_mav_veh_pitch - camera_fixed_angle;

        /* Calculate the x and z distances of the target relative to the drone camera (positive z is down).*/
        if (g_camera_tilt_angle <= 0.0f && g_camera_tilt_angle >= -g_camera_comp_angle)
        {
            camera_comp_angle_abs = std::fabs(g_camera_tilt_angle);

            g_x_target = g_d_target * cosf(camera_comp_angle_abs);
            g_z_target = g_d_target * sinf(camera_comp_angle_abs);
        }

        /* Calculate shift in target location in the x and z directions based on camera tilt
        angle and drone pitch. When the drone pitch is equal to the camera tilt angle,
        then there is no forward or backward adjustment of the target's position because
        the camera will be parallel to the ground.  This means there can only be an
        adjustment of the target position estimate in the z direction. Make adjustments when
        drone pitch is less than or equal to the camera tilt angle (brings the camera parallel
        to the ground) and greater than a calibratable value. */
        g_delta_angle = g_camera_comp_angle + g_mav_veh_pitch;

        if (g_delta_angle > 0.0f && g_delta_angle < camera_fixed_angle)
        {
            delta_idx_d = get_float_index(g_d_target, &delta_offset_d[0], MAX_IDX_DELTA_D_OFFSET, true);
            delta_idx_pix = get_float_index(std::fabs(g_target_cntr_offset_x), &delta_offset_pixels[0], MAX_IDX_DELTA_PIXEL_OFFSET, true);
            delta_d = get_2d_interpolated_value(&delta_offset[0][0], MAX_IDX_DELTA_PIXEL_OFFSET, MAX_IDX_DELTA_D_OFFSET, y_idx_pix, y_idx_d);

            g_delta_d_x = delta_d * cosf(g_delta_angle);
            g_delta_d_z = delta_d * sinf(g_delta_angle);

            /* If object center is below the center of the frame */
            if (g_target_cntr_offset_x > center_of_frame_height)
            {
                g_delta_d_x = -g_delta_d_x;
                g_delta_d_z = -g_delta_d_z;
            }
        }

        /* Adjust x and z distances based on camera tilt angle */
        g_x_target = g_x_target + g_delta_d_x;
        g_z_target = g_z_target + g_delta_d_z;

        /* Calculate target y offset from the center line of view of the camera based on calibrated forward distance and the
        offset of the center of the target to the side of the center of the video frame in pixels */
        y_idx_d = get_float_index(g_d_target, &y_offset_d[0], MAX_IDX_Y_D_OFFSET, true);
        y_idx_pix = get_float_index(std::fabs(g_target_cntr_offset_y), &y_offset_pixels[0], MAX_IDX_Y_PIXEL_OFFSET, true);
        float y_target_est = get_2d_interpolated_value(&y_offset[0][0], MAX_IDX_Y_PIXEL_OFFSET, MAX_IDX_Y_D_OFFSET, y_idx_pix, y_idx_d);

        g_y_target = y_target_est;

        if (g_target_cntr_offset_y < 0.0f)
        {
            g_y_target = -g_y_target;
        }

        x_target_prv = g_x_target;
        y_target_prv = g_y_target;
    }

    loc_target_valid_prv = g_target_valid;
}

/********************************************************************************
 * Function: update_target_location
 * Description: Update step in kalman filter.
 ********************************************************************************/
void update_target_location(void)
{
    if (g_target_data_useful && g_dt > 0.0001)
    {
        // Measurement vector (observations)
        colvec z(2);
        z << g_x_target << g_y_target;

        // Define the system transition matrix A_loc (discretized)
        // TODO: update the state transition matrix A_loc
        // Ensure g_dt is valid (std::isnan(g_dt) || std::isinf(g_dt) || g_dt <= 0)

        // No external control input for now
        colvec u(1, fill::zeros);

        // Perform Kalman filter update
        kf_loc.Kalmanf(z, u);

        // Retrieve the updated state
        colvec *state = kf_loc.GetCurrentEstimatedState();
        g_x_target_ekf = state->at(0);
        g_y_target_ekf = state->at(1);
        g_vx_target_ekf = state->at(2);
        g_vy_target_ekf = state->at(3);
        g_ax_target_ekf = state->at(4);
        g_ay_target_ekf = state->at(5);
    }
};

/********************************************************************************
 * Function: calc_vertical_fov
 * Description: Calculate the field of view of the camera in meters at the
 *              given vehicle height above the ground, and camera angle
 *              relative to the ground.
 ********************************************************************************/
void calc_vertical_fov(void)
{
    // Calculate the line of sight distance of the camera
    if ((g_camera_comp_angle + g_mav_veh_pitch) < (float)0.001)
    {
        g_line_of_sight = (float)0.0;
    }
    else
    {
        g_line_of_sight = std::fabs(g_mav_veh_local_ned_z) / cosf(g_camera_comp_angle + g_mav_veh_pitch);
    }

    // Calculate the pixel height of a known fixed object at given line of
    // sight distance from the camera.
    // The object isn't there, this is an intermediate calculation to determine
    // how many pixels correspond to a meter at the current drone height.
    const float PIX_HEIGHT_X = (float)1680.557;
    const float PIX_HEIGHT_X_POW = (float)(-0.863);
    // Height of a known object at given distance
    float ghost_object_height = PIX_HEIGHT_X * powf(g_line_of_sight, PIX_HEIGHT_X_POW);
    g_meter_per_pix = (float)1.778 / ghost_object_height;
    g_fov_height = g_meter_per_pix * g_input_video_height;
}

/********************************************************************************
 * Function: dtrmn_target_loc_img
 * Description: Determine target location and movement patterns in the image.
 ********************************************************************************/
void dtrmn_target_loc_img(void)
{
    if (!g_target_data_useful)
    {
        return;
    }

    // Moving average for more smoothing - modifies the history vector
    g_target_cntr_offset_x_mov_avg = moving_average(target_x_pix_hist4avg, g_target_cntr_offset_x, x_buffer_idx4avg, x_sum);
    g_target_cntr_offset_y_mov_avg = moving_average(target_y_pix_hist4avg, g_target_cntr_offset_y, y_buffer_idx4avg, y_sum);

    // Convert the
    g_target_cntr_offset_x_m = g_target_cntr_offset_x_mov_avg * g_meter_per_pix;
}

/********************************************************************************
 * Function: ~Localize
 * Description: Localize class destructor.
 ********************************************************************************/
Localize::~Localize(void) {};

/********************************************************************************
 * Function: init
 * Description: Initialize all Localize target variables.  Run once at the start
 *              of the program.
 ********************************************************************************/
bool Localize::init(void)
{
    get_localization_params();
    init_kf_loc();

    g_d_target_h = (float)0.0;
    g_d_target_w = (float)0.0;
    g_x_target = (float)0.0;
    g_y_target = (float)0.0;
    g_z_target = (float)0.0;
    g_d_target = (float)0.0;
    g_x_error = (float)0.0;
    g_y_error = (float)0.0;
    g_delta_angle = (float)0.0;
    g_camera_tilt_angle = (float)0.0;
    g_delta_d_x = (float)0.0;
    g_delta_d_z = (float)0.0;
    g_camera_comp_angle = (float)0.0;
    x_target_prv = (float)0.0;
    y_target_prv = (float)0.0;
    loc_target_valid_prv = false;
    g_target_data_useful = false;
    camera_comp_angle_abs = (float)0.0;
    g_fov_height = (float)0.0;
    g_meter_per_pix = (float)0.0;
    g_target_cntr_offset_x_m = (float)0.0;
    target_y_pix_hist4avg.assign(y_pix_window, (float)0.0);
    target_x_pix_hist4avg.assign(x_pix_window, (float)0.0);
    g_target_cntr_offset_x_mov_avg = (float)0.0;
    g_target_cntr_offset_y_mov_avg = (float)0.0;
    y_buffer_idx4avg = (int)0;
    y_sum = (float)0.0;
    x_buffer_idx4avg = (int)0;
    x_sum = (float)0.0;
    g_line_of_sight = (float)0.0;

    return true;
}

/********************************************************************************
 * Function: loop
 * Description: Return control parameters for the vehicle to Localize a designated
 *              target at a distance.
 ********************************************************************************/
void Localize::loop(void)
{
    dtrmn_target_location();
    update_target_location();
    calc_vertical_fov();
    dtrmn_target_loc_img();
}

/********************************************************************************
 * Function: shutdown
 * Description: Clean up code to run before program exits.
 ********************************************************************************/
void Localize::shutdown(void)
{
    // place clean up code here
}

#endif // ENABLE_CV
