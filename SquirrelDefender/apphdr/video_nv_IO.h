#pragma once

#ifdef ENABLE_CV
#ifdef JETSON_B01

/********************************************************************************
 * @file    video_nv_IO.h
 * @author  Cameron Rose
 * @date    6/7/2023
 ********************************************************************************/
#ifndef VIDEO_NV_IO_H
#define VIDEO_NV_IO_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "detect_target_jetson_inference.h"
#include <string>
#include <fstream>
#include "jetson-utils/videoSource.h"
#include "jetson-utils/videoOutput.h"
#include "jetson-inference/detectNet.h"
#include "jetson-inference/objectTracker.h"
#include <jetson-inference/objectTrackerIOU.h>


/********************************************************************************
 * Imported objects
 ********************************************************************************/
extern detectNet *net;

/********************************************************************************
 * Exported objects
 ********************************************************************************/
extern bool valid_image_rcvd;
extern videoSource *input;
extern uchar3 *image;
extern float input_video_width;
extern float input_video_height;

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
class VideoNV
{
public:
    VideoNV();
    ~VideoNV();

    static bool init(void);
    static void in_loop(void);
    static void out_loop(void);
    static void shutdown(void);

private:
};

#endif // VIDEO_NV_IO_H

#endif // JETSON_B01
#endif // ENABLE_CV
