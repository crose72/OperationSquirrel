#pragma once

/********************************************************************************
 * @file    datalog.h
 * @author  Cameron Rose
 * @date    1/22/2025
 ********************************************************************************/
#ifndef DATALOG_H
#define DATALOG_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include "mav_data_hub.h"
#include "path_planner.h"
#include "track_target.h"
#include "localize_target.h"
#include "detect_target.h"
#include "time_calc.h"

/********************************************************************************
 * Imported objects
 ********************************************************************************/
/* Mavlink variables */
extern float g_app_elapsed_time;

extern uint16_t g_mav_veh_sys_stat_voltage_battery;
extern int16_t g_mav_veh_sys_stat_current_battery;
extern int8_t g_mav_veh_sys_stat_battery_remaining;

extern int32_t g_mav_veh_rel_alt;
extern int16_t g_mav_veh_gps_vx;
extern int16_t g_mav_veh_gps_vy;
extern int16_t g_mav_veh_gps_vz;
extern uint16_t g_mav_veh_gps_hdg;

extern float g_mav_veh_roll;
extern float g_mav_veh_pitch;
extern float g_mav_veh_yaw;
extern float g_mav_veh_rollspeed;
extern float g_mav_veh_pitchspeed;
extern float g_mav_veh_yawspeed;
extern int16_t g_mav_veh_imu_ax;
extern int16_t g_mav_veh_imu_ay;
extern int16_t g_mav_veh_imu_az;
extern int16_t g_mav_veh_imu_xgyro;
extern int16_t g_mav_veh_imu_ygyro;
extern int16_t g_mav_veh_imu_zgyro;

extern uint16_t g_mav_veh_rngfdr_current_distance;
extern uint8_t g_mav_veh_rngfdr_signal_quality;

extern float g_mav_veh_flow_comp_m_x;
extern float g_mav_veh_flow_comp_m_y;
extern int16_t g_mav_veh_flow_x;
extern int16_t g_mav_veh_flow_y;
extern uint8_t g_mav_veh_flow_quality;
extern float g_mav_veh_flow_rate_x;
extern float g_mav_veh_flow_rate_y;

extern float g_mav_veh_local_ned_x;
extern float g_mav_veh_local_ned_y;
extern float g_mav_veh_local_ned_z;
extern float g_mav_veh_local_ned_vx;
extern float g_mav_veh_local_ned_vy;
extern float g_mav_veh_local_ned_vz;

extern float g_mav_veh_q1_actual;
extern float g_mav_veh_q2_actual;
extern float g_mav_veh_q3_actual;
extern float g_mav_veh_q4_actual;
extern float g_mav_veh_roll_rate_actual;
extern float g_mav_veh_pitch_rate_actual;
extern float g_mav_veh_yaw_rate_actual;
extern float g_mav_veh_repr_offset_q[4];

extern uint8_t g_mav_veh_type;
extern uint8_t g_mav_veh_autopilot_type;
extern uint8_t g_mav_veh_base_mode;
extern uint32_t g_mav_veh_custom_mode;
extern uint8_t g_mav_veh_state;
extern uint8_t g_mav_veh_mavlink_version;

/* Detect/Track target variables */
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
extern float g_detection_class;
extern float g_target_detection_conf;

/* Localize target variables*/
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
extern float g_x_target_ekf;
extern float g_y_target_ekf;
extern float g_vx_target_ekf;
extern float g_vy_target_ekf;
extern float g_ax_target_ekf;
extern float g_ay_target_ekf;
extern bool g_target_loc_data_ok;

/* Follow target variables */
extern bool g_target_too_close;
extern float g_vx_adjust;
extern float g_vy_adjust;
extern float g_vz_adjust;
extern float g_yaw_target;
extern float g_yaw_target_error;
extern float g_mav_veh_yaw_adjusted;

/* System variables */
extern SystemState g_system_state;

/********************************************************************************
 * Exported objects
 ********************************************************************************/

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/

class DataLogger
{
public:
    DataLogger();
    ~DataLogger();

    static bool init(void);
    static void loop(void);
    static void shutdown(void);

private:

};

#endif // DATALOG_H
