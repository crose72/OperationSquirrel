#pragma once

#ifdef ENABLE_CV
#ifdef BLD_JETSON_B01

/********************************************************************************
 * @file    video_io_nv.h
 * @author  Cameron Rose
 * @date    1/22/2025
 ********************************************************************************/
#ifndef VIDEO_IO_NV_H
#define VIDEO_IO_NV_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "detect_target.h"
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
extern detectNet *g_net;

/********************************************************************************
 * Exported objects
 ********************************************************************************/
extern bool g_valid_image_rcvd;
extern videoSource *g_input;
extern uchar3 *g_image;
extern float g_input_video_width;
extern float g_input_video_height;

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

#endif // VIDEO_IO_NV_H

#endif // BLD_JETSON_B01
#endif // ENABLE_CV
