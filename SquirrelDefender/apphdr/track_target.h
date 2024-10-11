#pragma once

#ifdef JETSON_B01

/********************************************************************************
 * @file    track_target.h
 * @author  Cameron Rose
 * @date    6/7/2023
 ********************************************************************************/
#ifndef TRACK_TARGET_H
#define TRACK_TARGET_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "video_IO.h"
#include "object_detection.h"
#include "parameters.h"
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/cudawarping.hpp>
#include <jetson-utils/cudaMappedMemory.h> // Assuming Jetson Inference utilities are available
#include <jetson-utils/cudaRGB.h>          // For cuda functions

/********************************************************************************
 * Imported objects
 ********************************************************************************/
extern detectNet *net;
extern detectNet::Detection *detections;
extern uchar3 *image;
extern int numDetections;
extern uint32_t input_video_width;
extern uint32_t input_video_height;
extern bool valid_image_rcvd;
extern bool target_identified;
extern int target_detection_ID;
extern int target_track_ID;
extern float object_height;
extern float object_width;
extern float object_aspect;
extern float object_left;
extern float object_right;
extern float object_top;
extern float object_bottom;

/********************************************************************************
 * Exported objects
 ********************************************************************************/

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
class Track
{
public:
    Track();
    ~Track();

    static bool init(void);
    static void loop(void);

private:
};

#endif // TRACK_TARGET_H

#endif // JETSON_B01