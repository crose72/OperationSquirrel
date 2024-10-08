#pragma once

#ifdef JETSON_B01

/********************************************************************************
 * @file    localize_target.h
 * @author  Cameron Rose
 * @date    6/7/2023
 ********************************************************************************/
#ifndef LOCALIZE_TARGET_H
#define LOCALIZE_TARGET_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "video_IO.h"
#include "object_detection.h"
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
extern float mav_veh_pitch;

/********************************************************************************
 * Exported objects
 ********************************************************************************/
extern bool target_identified;
extern int target_detection_ID;
extern int target_track_ID;
extern float center_offset_x;
extern float center_offset_y;
extern float object_height;
extern float object_width;
extern float object_aspect;
extern float object_left;
extern float object_right;
extern float object_top;
extern float object_bottom;
extern float d_object_h;
extern float d_object_w;
extern float x_object;
extern float y_object;
extern float z_object;
extern float d_object;
extern float x_error;
extern float y_error;
extern float delta_angle;
extern float camera_tilt_angle;
extern float delta_d_x;
extern float delta_d_z;
extern float camera_comp_angle;

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
class Localize
{
public:
    Localize();
    ~Localize();

    static bool init(void);
    static void loop(void);

    static void dtrmn_target(void);
    static void get_target_info(int n);
    static void calc_target_offest(void);

private:
};

#endif // LOCALIZE_TARGET_H

#endif // JETSON_B01