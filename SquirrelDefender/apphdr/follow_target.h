#pragma once

#ifdef USE_JETSON

/********************************************************************************
 * @file    follow_target.h
 * @author  Cameron Rose
 * @date    12/27/2023
 ********************************************************************************/
#ifndef FOLLOW_TARGET_H
#define FOLLOW_TARGET_H

/************************************
 * Includes
 ************************************/
#include "common_inc.h"
#include "videoIO.h"
#include "object_detection.h"
#include "vehicle_controller.h"
#include "velocity_controller.h"
#include "attitude_controller.h"
#include "parameters.h"

/************************************
 * Imported objects
 ************************************/
extern detectNet* net;
extern detectNet::Detection* detections;
extern videoSource* input;
extern uchar3* image;
extern int numDetections;
extern uint32_t input_video_width;
extern uint32_t input_video_height;

/************************************
 * Exported objects
 ************************************/

/************************************
 * Function prototypes
 ************************************/
class Follow
{
    public:
        Follow();
        ~Follow();

        static void follow_target_loop(void);
        static void overtake_target(void);
    
    private:
        static void get_control_params(void);
        static void get_target_desired_params(void);
        static void calc_target_actual_params(int n);
        static void calc_target_error(void);
        static void calc_overlap_error(void);
};

#endif // FOLLOW_TARGET_H

#endif // USE_JETSON