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
extern int32_t lat;
extern int32_t lon;
extern int32_t alt;
extern int32_t relative_alt;
extern int16_t vx;
extern int16_t vy;
extern int16_t vz;
extern uint16_t hdg;
extern float roll;
extern float pitch;
extern float yaw;
extern float rollspeed;
extern float pitchspeed;
extern float yawspeed;
extern int16_t accel_x;
extern int16_t accel_y;
extern int16_t accel_z;
extern int16_t gyro_x;
extern int16_t gyro_y;
extern int16_t gyro_z;
extern int16_t mag_x;
extern int16_t mag_y;
extern int16_t mag_z;

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