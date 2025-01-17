#pragma once

#ifdef ENABLE_CV

/********************************************************************************
 * @file    follow_target.h
 * @author  Cameron Rose
 * @date    6/7/2023
 ********************************************************************************/
#ifndef FOLLOW_TARGET_H
#define FOLLOW_TARGET_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "json_utils.h"
#include "pid.h"
#include "localize_target.h"

/********************************************************************************
 * Imported objects
 ********************************************************************************/
extern uint16_t mav_veh_rngfdr_min_distance;
extern uint16_t mav_veh_rngfdr_max_distance;
extern uint16_t g_mav_veh_rngfdr_current_distance;
extern float mav_veh_pitch;
extern bool g_target_valid;
extern float x_object;
extern float y_object;
extern float z_object;

/********************************************************************************
 * Exported objects
 ********************************************************************************/
extern bool g_target_too_close;
extern float x_error;
extern float y_error;
extern float g_vx_adjust;
extern float g_vy_adjust;
extern float g_vz_adjust;

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
class Follow
{
public:
    Follow();
    ~Follow();

    static void loop(void);
    static bool init(void);
    static void shutdown(void);

private:
};

#endif // FOLLOW_TARGET_H

#endif // ENABLE_CV
