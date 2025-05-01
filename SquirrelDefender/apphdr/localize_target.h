#pragma once

#ifdef ENABLE_CV

/********************************************************************************
 * @file    localize_target.h
 * @author  Cameron Rose
 * @date    1/22/2025
 ********************************************************************************/
#ifndef LOCALIZE_TARGET_H
#define LOCALIZE_TARGET_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "track_target.h"
#include "param_reader.h"
#include "interpolate.h"
#include "time_calc.h"
#include "kf.h"

/********************************************************************************
 * Imported objects
 ********************************************************************************/
extern float g_mav_veh_pitch;
extern bool g_target_valid;
extern int g_target_detection_id;
extern int g_target_track_id;
extern float g_target_cntr_offset_x;
extern float g_target_cntr_offset_y;
extern float g_target_height;
extern float g_target_width;
extern float g_target_aspect;
extern float g_target_left;
extern float g_target_right;
extern float g_target_top;
extern float g_target_bottom;
extern float g_target_center_x;
extern float g_target_center_y;
extern float g_dt;

/********************************************************************************
 * Exported objects
 ********************************************************************************/
extern float d_target_h;
extern float d_target_w;
extern float g_x_target;
extern float g_y_target;
extern float g_z_target;
extern float d_target;
extern float g_x_error;
extern float g_y_error;
extern float g_delta_angle;
extern float g_camera_tilt_angle;
extern float g_delta_d_x;
extern float g_delta_d_z;
extern float g_camera_comp_angle;
extern float g_x_target_ekf;
extern float g_y_target_ekf;
extern float g_vx_target_ekf;
extern float g_vy_target_ekf;
extern float g_ax_target_ekf;
extern float g_ay_target_ekf;
extern bool g_target_loc_data_ok;

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
class Localize
{
public:
    Localize();
    ~Localize();

    static bool init(void);
    static void loop(void);
    static void shutdown(void);

private:
};

#endif // LOCALIZE_TARGET_H

#endif // ENABLE_CV
