/********************************************************************************
 * @file    datalog.h
 * @author  Cameron Rose
 * @date    12/27/2023
 ********************************************************************************/
#ifndef DATALOG_H
#define DATALOG_H

/************************************
 * Includes
 ************************************/
#include "common_inc.h"

/************************************
 * Imported objects
 ************************************/
extern float elapsedTimeSeconds;
extern int32_t mav_veh_lat;
extern int32_t mav_veh_lon;
extern int32_t mav_veh_alt;
extern int32_t mav_rel_alt;
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

/************************************
 * Exported objects
 ************************************/

/************************************
 * Function prototypes
 ************************************/
template <typename T>
std::string toString(const T& value);
void logData(void);
void writeToCSV(const std::string& filename, const std::vector<std::vector<std::string>>& data);
std::string checkAndAppendFileName(const std::string& filename);

#endif // DATALOG_H