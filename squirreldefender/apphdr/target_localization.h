#pragma once

/********************************************************************************
 * @file    target_localization.h
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Target localization module that converts image detections into
 *          line-of-sight vectors, metric position estimates, and derived
 *          kinematics. Provides initialization, per-frame update, and cleanup
 *          for the localization pipeline.
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
extern float g_tgt_los_dist_from_pix_height;
extern float g_tgt_los_dist_from_pix_width;
extern float g_tgt_pos_x_meas;
extern float g_tgt_pos_y_meas;
extern float g_tgt_pos_z_meas;
extern float g_tgt_los_dist_meas;
extern float g_cam0_delta_angle_rad;
extern float g_cam0_angle_rad;
extern float g_tgt_pos_x_delta;
extern float g_tgt_pos_z_delta;
extern float g_cam0_comp_angle_rad;
extern float g_tgt_pos_x_est;
extern float g_tgt_pos_y_est;
extern float g_tgt_vel_x_est;
extern float g_tgt_vel_y_est;
extern float g_tgt_acc_x_est;
extern float g_tgt_acc_y_est;
extern bool g_tgt_meas_valid;
extern float g_cam0_fov_height;
extern float g_cam0_m_per_pix;
extern float g_tgt_cntr_offset_x_m;
extern float g_tgt_cntr_offset_x_pix_filt;
extern float g_tgt_cntr_offset_y_pix_filt;
extern float g_cam0_los_m;
extern bool g_tgt_lost;
extern float g_tgt_lost_dbc_sec;

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
class TargetLocalization
{
public:
    TargetLocalization();
    ~TargetLocalization();

    static bool init(void);
    static void loop(void);
    static void shutdown(void);

private:
};

#endif // LOCALIZE_TARGET_H

#endif // ENABLE_CV
