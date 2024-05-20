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
extern float roll;
extern float pitch;
extern float yaw;
extern float rollspeed;
extern float pitchspeed;
extern float yawspeed;
extern int16_t xacc;
extern int16_t yacc;
extern int16_t zacc;
extern int16_t xgyro;
extern int16_t ygyro;
extern int16_t zgyro;
extern int16_t xmag;
extern int16_t ymag;
extern int16_t zmag;

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