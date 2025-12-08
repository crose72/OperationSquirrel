/********************************************************************************
 * @file    global_calibrations.cpp
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Defines system-wide calibration constants used throughout the
 *          embedded system. These include MAVLink system/component identifiers
 *          and other configuration parameters that must remain consistent across
 *          all modules.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "global_calibrations.h"

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/
const uint8_t SENDER_SYS_ID = (uint8_t)0;  // MAVLink system ID for the Jetson sender
const uint8_t SENDER_COMP_ID = (uint8_t)0; // MAVLink component ID for the Jetson sender
const uint8_t TARGET_SYS_ID = (uint8_t)1;  // MAVLink system ID for the flight controller
const uint8_t TARGET_COMP_ID = (uint8_t)1; // MAVLink component ID for the flight controller
