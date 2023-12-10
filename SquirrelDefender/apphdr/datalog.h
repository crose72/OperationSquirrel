#ifndef DATALOG_H
#define DATALOG_H

#include "standard_libs.h"

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
extern int16_t xacc;
extern int16_t yacc;
extern int16_t zacc;
extern int16_t xgyro;
extern int16_t ygyro;
extern int16_t zgyro;
extern int16_t xmag;
extern int16_t ymag;
extern int16_t zmag;

template <typename T>
std::string toString(const T& value);

void logData(void);
void writeToCSV(const std::string& filename, const std::vector<std::vector<std::string>>& data);
std::string checkAndAppendFileName(const std::string& filename);

#endif // DATALOG_H