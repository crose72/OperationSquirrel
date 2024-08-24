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
#include "follow_target.h"

/********************************************************************************
 * Imported objects
 ********************************************************************************/
extern float app_elapsed_time;

extern uint16_t mav_veh_sys_stat_voltage_battery;
extern int16_t mav_veh_sys_stat_current_battery;
extern int8_t mav_veh_sys_stat_battery_remaining;

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

extern uint16_t mav_veh_rngfdr_current_distance;
extern uint8_t mav_veh_rngfdr_signal_quality;

extern float mav_veh_flow_comp_m_x;
extern float mav_veh_flow_comp_m_y;
extern int16_t mav_veh_flow_x;
extern int16_t mav_veh_flow_y;
extern uint8_t mav_veh_quality;
extern float mav_veh_flow_rate_x;
extern float mav_veh_flow_rate_y;

extern float mav_veh_local_ned_x;
extern float mav_veh_local_ned_y;
extern float mav_veh_local_ned_z;
extern float mav_veh_local_ned_vx;
extern float mav_veh_local_ned_vy;
extern float mav_veh_local_ned_vz;

extern float mav_veh_q1_actual;
extern float mav_veh_q2_actual;
extern float mav_veh_q3_actual;
extern float mav_veh_q4_actual;
extern float mav_veh_roll_rate_actual;
extern float mav_veh_pitch_rate_actual;
extern float mav_veh_yaw_rate_actual;
extern float mav_veh_repr_offset_q[4];

extern bool target_too_close;
extern bool target_identified;
extern float vx_adjust;
extern float vy_adjust;
extern float vz_adjust;

extern float x_actual;
extern float height_actual;
extern float y_actual;
extern float width_actual;
extern float x_centroid_err;
extern float target_height_err;
extern float y_centroid_err;
extern float target_left_side;
extern float target_right_side;
extern float target_left_err;
extern float target_right_err;
extern float target_height_err_rev;

extern SYSTEM_STATE system_state;

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