#pragma once

/********************************************************************************
 * @file    localize_target.h
 * @author  Cameron Rose
 * @date    1/22/2025
 ********************************************************************************/
#ifdef ENABLE_CV

#ifndef LOCALIZE_TARGET_H
#define LOCALIZE_TARGET_H

/********************************************************************************
 * Includes
 ********************************************************************************/

/********************************************************************************
 * Exported objects
 ********************************************************************************/
extern float g_d_target_h;
extern float g_d_target_w;
extern float g_x_target;
extern float g_y_target;
extern float g_z_target;
extern float g_d_target;
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
extern bool g_target_data_useful;
extern float g_fov_height;
extern float g_meter_per_pix;
extern float g_target_cntr_offset_x_m;
extern float g_target_cntr_offset_x_mov_avg;
extern float g_target_cntr_offset_y_mov_avg;
extern float g_line_of_sight;

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
