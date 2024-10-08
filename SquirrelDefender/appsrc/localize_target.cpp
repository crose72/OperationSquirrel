#ifdef JETSON_B01

/********************************************************************************
 * @file    localize_target.cpp
 * @author  Cameron Rose
 * @date    6/7/2023
 * @brief   Localize the target and maintain a specified x, y, z offset.
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
DebugTerm LocalizeData("");

bool target_identified;
int target_detection_ID;
int target_track_ID;
float center_offset_x;
float center_offset_y;
float object_height;
float object_width;
float object_aspect;
float object_left;
float object_right;
float object_top;
float object_bottom;
float d_object_h;
float d_object_w;
float x_object;
float y_object;
float z_object;
float d_object;
float delta_angle;       //
float camera_tilt_angle; // Angle of the camera relative to the ground, compensating for pitch
float delta_d_x;
float delta_d_z;
float camera_comp_angle;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/
const float center_of_frame_width = 640.0f;
const float center_of_frame_height = 360.0f;
const float camera_fixed_angle = 0.426f; // radians
const float PI = 3.14159;

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

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: Localize
 * Description: Localize class constructor.
 ********************************************************************************/
Localize::Localize(void) {};

/********************************************************************************
 * Function: ~Localize
 * Description: Localize class destructor.
 ********************************************************************************/
Localize::~Localize(void) {};

/********************************************************************************
 * Function: get_target_info
 * Description: Obtain the important information about the target, such as the
 *              height and location of the center of the target in the frame.
 ********************************************************************************/
void Localize::get_target_info(int n)
{
    float object_center_y = (detections[n].Left + detections[n].Right) / 2.0f;
    float object_center_x = (detections[n].Bottom + detections[n].Top) / 2.0f;

    target_track_ID = detections[n].TrackID;
    center_offset_y = object_center_y - center_of_frame_width;
    center_offset_x = object_center_x - center_of_frame_height;
    object_height = detections[n].Height();
    object_width = detections[n].Width();
    object_aspect = object_width / object_height;
    object_left = detections[n].Left;
    object_right = detections[n].Right;
    object_top = detections[n].Top;
    object_bottom = detections[n].Bottom;
}

/********************************************************************************
 * Function: calc_target_offest
 * Description: Calculate the location of the target relative to the drone.
 *              First implementation will be relative to the camera, needs to use
 *              the center mass of the drone in the end though.
 ********************************************************************************/
void Localize::calc_target_offest(void)
{
    float d_idx_h;
    float d_idx_w;
    float y_idx_d;
    float y_idx_pix;
    float delta_idx_d;
    float delta_idx_pix;
    float delta_d;

    /* Calculate distance from camera offset */
    d_idx_h = get_float_index(object_height, &height_index[0], MAX_IDX_D_HEIGHT, false);
    d_idx_w = get_float_index(object_width, &width_index[0], MAX_IDX_D_WIDTH, false);
    d_object_h = get_interpolated_value(d_idx_h, &d_offset_h[0], MAX_IDX_D_HEIGHT);
    d_object_w = get_interpolated_value(d_idx_w, &d_offset_w[0], MAX_IDX_D_WIDTH);

    if (object_aspect > 0.4f)
    {
        d_object = d_object_w;
    }
    else if (d_object_h > 2.99f)
    {
        d_object = d_object_h;
    }
    else
    {
        d_object = d_object_w;
    }

    /* Calculate true camera angle relative to the ground, adjusting for pitch. */
    camera_comp_angle = (PI / 2.0f) - camera_fixed_angle;
    camera_tilt_angle = mav_veh_pitch - camera_fixed_angle;

    /* Calculate the x and z distances of the target relative to the drone camera (positive z is down).*/
    if (camera_tilt_angle <= 0.0f && camera_tilt_angle >= -camera_comp_angle)
    {
        float camera_comp_angle_abs = abs(camera_tilt_angle);

        x_object = d_object * cos(camera_comp_angle_abs);
        z_object = d_object * sin(camera_comp_angle_abs);
    }

    /* Calculate shift in target location in the x and z directions based on camera tilt
       angle and drone pitch. When the drone pitch is equal to the camera tilt angle,
       then there is no forward or backward adjustment of the target's position because
       the camera will be parallel to the ground.  This means there can only be an
       adjustment of the target position estimate in the z direction. Make adjustments when
       drone pitch is less than or equal to the camera tilt angle (brings the camera parallel
       to the ground) and greater than a calibratable value. */
    delta_angle = camera_comp_angle + mav_veh_pitch;

    if (delta_angle > 0.0f && delta_angle < camera_fixed_angle)
    {
        delta_idx_d = get_float_index(d_object, &delta_offset_d[0], MAX_IDX_DELTA_D_OFFSET, true);
        delta_idx_pix = get_float_index(std::abs(center_offset_x), &delta_offset_pixels[0], MAX_IDX_DELTA_PIXEL_OFFSET, true);
        delta_d = get_2d_interpolated_value(&delta_offset[0][0], MAX_IDX_DELTA_PIXEL_OFFSET, MAX_IDX_DELTA_D_OFFSET, y_idx_pix, y_idx_d);

        delta_d_x = delta_d * cos(delta_angle);
        delta_d_z = delta_d * sin(delta_angle);

        /* If object center is below the center of the frame */
        if (center_offset_x > center_of_frame_height)
        {
            delta_d_x = -delta_d_x;
            delta_d_z = -delta_d_z;
        }
    }

    /* Adjust x and z distances based on camera tilt angle */
    x_object = x_object + delta_d_x;
    x_object = x_object + delta_d_x;

    /* Calculate target y offset from the center line of view of the camera based on calibrated forward distance and the
       offset of the center of the target to the side of the center of the video frame in pixels */
    y_idx_d = get_float_index(d_object, &y_offset_d[0], MAX_IDX_Y_D_OFFSET, true);
    y_idx_pix = get_float_index(std::abs(center_offset_y), &y_offset_pixels[0], MAX_IDX_Y_PIXEL_OFFSET, true);
    y_object = get_2d_interpolated_value(&y_offset[0][0], MAX_IDX_Y_PIXEL_OFFSET, MAX_IDX_Y_D_OFFSET, y_idx_pix, y_idx_d);

    if (center_offset_y < 0.0f)
    {
        y_object = -y_object;
    }
}

/********************************************************************************
 * Function: dtrmn_target
 * Description: Determine which detected object to Localize.
 ********************************************************************************/
void Localize::dtrmn_target(void)
{
    target_detection_ID = -1;

    for (int n = 0; n < numDetections; n++)
    {
        /* A tracked object, classified as a person with some confidence level */
        if (detections[n].TrackID >= 0 && detections[n].ClassID == 1 && detections[n].Confidence > 0.5)
        {
            target_detection_ID = n;
        }
    }

    get_target_info(target_detection_ID);

    /* Target detected, tracked, and has a size greater than 0 */
    if (target_detection_ID >= 0 && target_track_ID >= 0 && object_height > 1 && object_width > 1)
    {
        target_identified = true;
    }
    else
    {
        target_identified = false;
    }
}

/********************************************************************************
 * Function: localize_target_init
 * Description: Initialize all Localize target variables.  Run once at the start
 *              of the program.
 ********************************************************************************/
bool Localize::init(void)
{
    target_identified = false;
    center_offset_x = 0.0f;
    center_offset_y = 0.0f;
    object_height = 0.0f;
    object_width = 0.0f;
    object_aspect = 0.0f;
    object_left = 0.0f;
    object_right = 0.0f;
    object_top = 0.0f;
    object_bottom = 0.0f;
    d_object_h = 0.0f;
    d_object_w = 0.0f;
    x_object = 0.0f;
    y_object = 0.0f;
    z_object = 0.0f;
    d_object = 0.0f;
    x_error = 0.0f;
    y_error = 0.0f;
    delta_angle = 0.0f;
    camera_tilt_angle = 0.0f;
    delta_d_x = 0.0f;
    delta_d_z = 0.0f;
    camera_comp_angle = 0.0f;
    target_detection_ID = -1;

    return true;
}

/********************************************************************************
 * Function: localize_control_loop
 * Description: Return control parameters for the vehicle to Localize a designated
 *              target at a distance.
 ********************************************************************************/
void Localize::loop(void)
{
    dtrmn_target();

    if (target_identified)
    {
        calc_target_offest();
    }
}

#endif // JETSON_B01