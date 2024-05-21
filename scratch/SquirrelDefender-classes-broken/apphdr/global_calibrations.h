/********************************************************************************
 * @file    global_calibrations.h
 * @author  Cameron Rose
 * @date    12/27/2023
 ********************************************************************************/
#ifndef GLOBAL_CALIBRATIONS_H
#define GLOBAL_CALIBRATIONS_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"

/********************************************************************************
 * Global calibration declarations
 ********************************************************************************/
extern const uint8_t SENDER_SYS_ID;
extern const uint8_t SENDER_COMP_ID;
extern const uint8_t TARGET_SYS_ID;
extern const uint8_t TARGET_COMP_ID;
extern const int32_t MESSAGE_RATE_DEFAULT; // microseconds
extern const int32_t MESSAGE_RATE_1000us;

#endif // GLOBAL_CALIBRATIONS_H