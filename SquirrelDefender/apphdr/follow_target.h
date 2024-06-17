#pragma once

#ifdef USE_JETSON

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
extern detectNet* net;
extern detectNet::Detection* detections;
extern videoSource* input;
extern int numDetections;
extern uint32_t input_video_width;
extern uint32_t input_video_height;

/********************************************************************************
 * Exported objects
 ********************************************************************************/

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
class Follow
{
    public:
        Follow();
        ~Follow();

        static void follow_target_loop(void);
        static bool follow_target_init(void);
    
    private:
    	static int dtrmn_target_ID(void);
        static void get_control_params(void);
        static void get_desired_target_size(void);
        static void calc_target_size(int n);
        static void calc_follow_error(void);
        static void overtake_target(void);
};

#endif // FOLLOW_TARGET_H

#endif // USE_JETSON