#pragma once

/********************************************************************************
 * @file    mav_data_hub.h
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Centralized hub of MAVLink-related global data fields used across the
 *          software system. This file provides extern declarations only; values
 *          are owned/updated by the MAVLink receiver module.
 ********************************************************************************/
#ifndef MAV_DATA_HUB_H
#define MAV_DATA_HUB_H

/********************************************************************************
 * Includes
 ********************************************************************************/

/********************************************************************************
 * Exported objects
 ********************************************************************************/

// Command / ACK state
extern uint16_t g_mav_cmd_id;
extern uint8_t g_mav_cmd_result;
extern uint8_t g_mav_cmd_progress;
extern int32_t g_mav_cmd_result_param2;
extern uint8_t g_mav_cmd_tgt_sys;
extern uint8_t g_mav_cmd_tgt_comp;

// System status fields
extern uint32_t g_mav_sys_sensors_present;
extern uint32_t g_mav_sys_sensors_enabled;
extern uint32_t g_mav_sys_sensors_health;
extern uint16_t g_mav_sys_load;
extern uint16_t g_mav_batt_voltage_mv;
extern int16_t g_mav_batt_current_ma;
extern uint16_t g_mav_comm_drop_rate;
extern uint16_t g_mav_comm_errors;
extern uint16_t g_mav_err_count1;
extern uint16_t g_mav_err_count2;
extern uint16_t g_mav_err_count3;
extern uint16_t g_mav_err_count4;
extern int8_t g_mav_batt_remaining_pct;
extern uint32_t g_mav_sys_sensors_present_ext;
extern uint32_t g_mav_sys_sensors_enabled_ext;
extern uint32_t g_mav_sys_sensors_health_ext;

// Flight mode / vehicle info
extern uint32_t g_mav_mode_custom;
extern uint8_t g_mav_type;
extern uint8_t g_mav_autopilot_type;
extern uint8_t g_mav_mode_base;
extern uint8_t g_mav_state;
extern uint8_t g_mav_version;

// GPS
extern int32_t g_mav_gps_lat;
extern int32_t g_mav_gps_lon;
extern int32_t g_mav_gps_alt_msl;
extern int32_t g_mav_gps_alt_rel;
extern int16_t g_mav_gps_vel_x;
extern int16_t g_mav_gps_vel_y;
extern int16_t g_mav_gps_vel_z;
extern uint16_t g_mav_gps_heading_cdeg;

// Attitude
extern float g_mav_veh_roll_rad;
extern float g_mav_veh_pitch_rad;
extern float g_mav_veh_yaw_rad;
extern float g_mav_veh_roll_rate;
extern float g_mav_veh_pitch_rate;
extern float g_mav_veh_yaw_rate;

// IMU
extern int16_t g_mav_imu_accel_x;
extern int16_t g_mav_imu_accel_y;
extern int16_t g_mav_imu_accel_z;
extern int16_t g_mav_imu_gyro_x;
extern int16_t g_mav_imu_gyro_y;
extern int16_t g_mav_imu_gyro_z;
extern int16_t g_mav_imu_mag_x;
extern int16_t g_mav_imu_mag_y;
extern int16_t g_mav_imu_mag_z;

// Attitude targets
extern float g_mav_att_target_q1;
extern float g_mav_att_target_q2;
extern float g_mav_att_target_q3;
extern float g_mav_att_target_q4;
extern float g_mav_att_target_roll_rate;
extern float g_mav_att_target_pitch_rate;
extern float g_mav_att_target_yaw_rate;
extern float g_mav_att_target_thrust;

// Actual (EKF) attitude
extern float g_mav_att_actual_q1;
extern float g_mav_att_actual_q2;
extern float g_mav_att_actual_q3;
extern float g_mav_att_actual_q4;
extern float g_mav_att_actual_roll_rate;
extern float g_mav_att_actual_pitch_rate;
extern float g_mav_att_actual_yaw_rate;
extern float g_mav_att_repr_offset_q[4];

// Rangefinder
extern uint16_t g_mav_rngfndr_min_cm;
extern uint16_t g_mav_rngfndr_max_cm;
extern uint16_t g_mav_rngfndr_dist_cm;
extern uint8_t g_mav_rngfndr_type;
extern uint8_t g_mav_rngfndr_id;
extern uint8_t g_mav_rngfndr_orient;
extern uint8_t g_mav_rngfndr_cov;
extern float g_mav_rngfndr_fov_horiz_rad;
extern float g_mav_rngfndr_fov_vert_rad;
extern float g_mav_rngfndr_quat[4];
extern uint8_t g_mav_rngfndr_quality;

// Optical flow
extern float g_mav_flow_vel_x;
extern float g_mav_flow_vel_y;
extern float g_mav_flow_ground_dist_m;
extern int16_t g_mav_flow_px_x;
extern int16_t g_mav_flow_px_y;
extern uint8_t g_mav_flow_sensor_id;
extern uint8_t g_mav_flow_quality;
extern float g_mav_flow_rate_x;
extern float g_mav_flow_rate_y;

// NED pose/velocity
extern float g_mav_veh_pos_ned_x;
extern float g_mav_veh_pos_ned_y;
extern float g_mav_veh_pos_ned_z;
extern float g_mav_veh_vel_ned_x;
extern float g_mav_veh_vel_ned_y;
extern float g_mav_veh_vel_ned_z;

// Parameters
extern float g_mav_param_val;
extern uint16_t g_mav_param_count;
extern uint16_t g_mav_param_index;
extern char g_mav_param_id[17];
extern uint8_t g_mav_param_type;
extern std::string g_mav_param_name;
extern bool g_mav_param_read;

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/

class MavMsg
{
public:
    MavMsg();
    ~MavMsg();

    static bool init(void);
    static void loop(void);
    static void shutdown(void);

private:
};

#endif // MAV_DATA_HUB_H
