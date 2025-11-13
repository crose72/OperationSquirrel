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
#include <chrono>

/********************************************************************************
 * Exported objects
 ********************************************************************************/
extern float g_tgt_height_meas;
extern float g_tgt_width_meas;
extern float g_tgt_pos_x_meas;
extern float g_tgt_pos_y_meas;
extern float g_tgt_pos_z_meas;
extern float g_tgt_dist_meas;
extern float g_pos_err_x;
extern float g_pos_err_y;
extern float g_cam_delta_angle_deg;
extern float g_cam_tilt_deg;
extern float g_dist_delta_x;
extern float g_dist_delta_z;
extern float g_cam_comp_angle_deg;
extern float g_tgt_pos_x_est;
extern float g_tgt_pos_y_est;
extern float g_tgt_vel_x_est;
extern float g_tgt_vel_y_est;
extern float g_tgt_acc_x_est;
extern float g_tgt_acc_y_est;
extern bool g_tgt_meas_valid;
extern float g_cam_fov_height;
extern float g_m_per_pix;
extern float g_tgt_cntr_offset_x_m;
extern float g_tgt_cntr_offset_x_filt;
extern float g_tgt_cntr_offset_y_filt;
extern float g_los_m;
extern bool g_tgt_lost;
extern float g_tgt_lost_dbc_sec;

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
