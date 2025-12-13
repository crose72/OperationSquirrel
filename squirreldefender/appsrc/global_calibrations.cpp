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
const uint8_t companion_sys_id = (uint8_t)0;  // MAVLink system ID for the Jetson sender
const uint8_t companion_comp_id = (uint8_t)0; // MAVLink component ID for the Jetson sender
const uint8_t autopilot_sys_id = (uint8_t)1;  // MAVLink system ID for the flight controller
const uint8_t autopilot_comp_id = (uint8_t)1; // MAVLink component ID for the flight controller
