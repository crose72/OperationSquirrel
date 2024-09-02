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
#include "video_IO.h"
#include "object_detection.h"
#include "vehicle_controller.h"
#include "velocity_controller.h"
#include "attitude_controller.h"
#include "pid_controller.h"
#include "parameters.h"

/********************************************************************************
 * Imported objects
 ********************************************************************************/
extern detectNet *net;
extern detectNet::Detection *detections;
extern videoSource *input;
extern int numDetections;
extern uint32_t input_video_width;
extern uint32_t input_video_height;
extern uint16_t mav_veh_rngfdr_min_distance;
extern uint16_t mav_veh_rngfdr_max_distance;
extern uint16_t mav_veh_rngfdr_current_distance;

/********************************************************************************
 * Exported objects
 ********************************************************************************/
extern bool target_too_close;
extern bool target_identified;
extern float vx_adjust;
extern float vy_adjust;
extern float vz_adjust;
extern float x_actual;
extern float height_actual;
extern float y_actual;
extern float width_actual;
extern float x_centroid_err;
extern float target_height_err;
extern float y_centroid_err;
extern float target_left_side;
extern float target_right_side;
extern float target_left_err;
extern float target_right_err;
extern float target_height_err_rev;

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
class Follow
{
public:
    Follow();
    ~Follow();

    static void follow_control_loop(void);
    static bool follow_target_init(void);

    static int dtrmn_target_ID(void);
    static void get_control_params(void);
    static void get_desired_target_size(void);
    static void calc_target_size(int n);
    static void calc_follow_error(void);
    static void overtake_target(void);

private:
    static float error_zero_protection(float desired, float actual, float threshold);
};

#endif // FOLLOW_TARGET_H

#endif // JETSON_B01