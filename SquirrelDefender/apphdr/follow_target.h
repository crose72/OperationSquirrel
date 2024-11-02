#pragma once

#ifdef JETSON_B01

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
#include "parameters.h"
#include "pid_controller.h"
#include "localize_target.h"

/********************************************************************************
 * Imported objects
 ********************************************************************************/
extern uint16_t mav_veh_rngfdr_min_distance;
extern uint16_t mav_veh_rngfdr_max_distance;
extern uint16_t mav_veh_rngfdr_current_distance;
extern float mav_veh_pitch;
extern bool target_valid;
extern float x_object;
extern float y_object;
extern float z_object;

/********************************************************************************
 * Exported objects
 ********************************************************************************/
extern bool target_too_close;
extern float x_error;
extern float y_error;
extern float vx_adjust;
extern float vy_adjust;
extern float vz_adjust;

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

#endif // JETSON_B01
