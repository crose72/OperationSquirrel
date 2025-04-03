#pragma once

/********************************************************************************
 * @file    global_calibrations.h
 * @author  Cameron Rose
 * @date    1/22/2025
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
extern const int32_t MESSAGE_RATE_DEFAULT;
extern const int32_t MESSAGE_RATE_1Hz;
extern const int32_t MESSAGE_RATE_40Hz;
extern const float PI;

#endif // GLOBAL_CALIBRATIONS_H
