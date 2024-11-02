#pragma once

#ifdef ENABLE_CV

/********************************************************************************
 * @file    video_IO.h
 * @author  Cameron Rose
 * @date    6/7/2023
 ********************************************************************************/
#ifndef VIDEO_IO_H
#define VIDEO_IO_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "jetson-utils/videoSource.h"
#include "jetson-utils/videoOutput.h"
#include "jetson-inference/detectNet.h"
#include "jetson-inference/objectTracker.h"
#include <jetson-inference/objectTrackerIOU.h>
#include "detect_target.h"
#include <string>
#include <fstream>

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
extern uint32_t input_video_width;
extern uint32_t input_video_height;

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
class Video
{
public:
    Video();
    ~Video();

    static bool init(void);
    static void in_loop(void);
    static void out_loop(void);
    static void shutdown(void);

private:
};

#endif // VIDEO_IO_H

#endif // ENABLE_CV
