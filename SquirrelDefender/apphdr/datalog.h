#pragma once

/********************************************************************************
 * @file    datalog.h
 * @author  Cameron Rose
 * @date    6/7/2023
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
#include "mavlink_msg_handler.h"

/********************************************************************************
 * Imported objects
 ********************************************************************************/
extern float app_elapsed_time;
extern uint16_t mav_veh_command_id;
extern uint8_t mav_veh_command_result;
extern uint8_t mav_veh_command_progress;
extern int32_t mav_veh_command_result_param2;
extern uint8_t mav_veh_command_target_system;
extern uint8_t mav_veh_command_target_component;

extern uint32_t mav_veh_sys_stat_onbrd_cntrl_snsrs_present;
extern uint32_t mav_veh_sys_stat_onbrd_cntrl_snsrs_enabled;
extern uint32_t mav_veh_sys_stat_onbrd_cntrl_snsrs_health;
extern uint16_t mav_veh_sys_stat_load;
extern uint16_t mav_veh_sys_stat_voltage_battery;
extern int16_t mav_veh_sys_stat_current_battery;
extern uint16_t mav_veh_sys_stat_drop_rate_comm;
extern uint16_t mav_veh_sys_stat_errors_comm;
extern uint16_t mav_veh_sys_stat_errors_count1;
extern uint16_t mav_veh_sys_stat_errors_count2;
extern uint16_t mav_veh_sys_stat_errors_count3;
extern uint16_t mav_veh_sys_stat_errors_count4;
extern int8_t mav_veh_sys_stat_battery_remaining;
extern uint32_t mav_veh_sys_stat_onbrd_cntrl_snsrs_prsnt_extnd;
extern uint32_t mav_veh_sys_stat_onbrd_cntrl_snsrs_enbld_extnd;
extern uint32_t mav_veh_sys_stat_onbrd_cntrl_snsrs_health_extnd;

extern uint32_t mav_veh_custom_mode;
extern uint8_t mav_veh_type;
extern uint8_t mav_veh_autopilot_type;
extern uint8_t mav_veh_base_mode;
extern uint8_t mav_veh_state;
extern uint8_t mav_veh_mavlink_version;

extern int32_t mav_veh_lat;
extern int32_t mav_veh_lon;
extern int32_t mav_veh_alt;
extern int32_t mav_veh_rel_alt;
extern int16_t mav_veh_gps_vx;
extern int16_t mav_veh_gps_vy;
extern int16_t mav_veh_gps_vz;
extern uint16_t mav_veh_gps_hdg;

extern float mav_veh_roll;
extern float mav_veh_pitch;
extern float mav_veh_yaw;
extern float mav_veh_rollspeed;
extern float mav_veh_pitchspeed;
extern float mav_veh_yawspeed;
extern int16_t mav_veh_imu_ax;
extern int16_t mav_veh_imu_ay;
extern int16_t mav_veh_imu_az;
extern int16_t mav_veh_imu_xgyro;
extern int16_t mav_veh_imu_ygyro;
extern int16_t mav_veh_imu_zgyro;
extern int16_t mav_veh_imu_xmag;
extern int16_t mav_veh_imu_ymag;
extern int16_t mav_veh_imu_zmag;

extern float mav_veh_q1_target;
extern float mav_veh_q2_target;
extern float mav_veh_q3_target;
extern float mav_veh_q4_target;
extern float mav_veh_roll_rate_target;
extern float mav_veh_pitch_rate_target;
extern float mav_veh_yaw_rate_target;
extern float mav_veh_thrust_target;

extern float mav_veh_q1_actual;
extern float mav_veh_q2_actual;
extern float mav_veh_q3_actual;
extern float mav_veh_q4_actual;
extern float mav_veh_roll_rate_actual;
extern float mav_veh_pitch_rate_actual;
extern float mav_veh_yaw_rate_actual;
extern float mav_veh_thrust_actual;

extern uint16_t mav_veh_rngfdr_min_distance;
extern uint16_t mav_veh_rngfdr_max_distance;
extern uint16_t mav_veh_rngfdr_current_distance;
extern uint8_t mav_veh_rngfdr_type;
extern uint8_t mav_veh_rngfdr_id;
extern uint8_t mav_veh_rngfdr_orientation;
extern uint8_t mav_veh_rngfdr_covariance;
extern float mav_veh_rngfdr_horizontal_fov;
extern float mav_veh_rngfdr_vertical_fov;
extern float mav_veh_rngfdr_quaternion[4];
extern uint8_t mav_veh_rngfdr_signal_quality;

extern float mav_veh_flow_comp_m_x;
extern float mav_veh_flow_comp_m_y;
extern float mav_veh_ground_distance;
extern int16_t mav_veh_flow_x;
extern int16_t mav_veh_flow_y;
extern uint8_t mav_veh_sensor_id;
extern uint8_t mav_veh_quality;
extern float mav_veh_flow_rate_x;
extern float mav_veh_flow_rate_y;

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

    static bool data_log_init(void);
    static void data_log_loop(void);
    static void data_log_shutdown(void);

private:
    static void save_to_csv(const std::string &filename, const std::vector<std::vector<std::string>> &data);
    static std::string generate_unique_filename(const std::string &filename);
};

#endif // DATALOG_H