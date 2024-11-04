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
#include <string>
#include <fstream>

#ifdef JETSON_B01

#include "video_nv_IO.h"

#elif _WIN32

#include "video_win_IO.h"

#else

#error "Please define a build platform."

#endif // JETSON_B01


/********************************************************************************
 * Imported objects
 ********************************************************************************/
#ifdef JETSON_B01

extern detectNet *net;

#elif _WIN32

/* No imported objects */

#else

#error "Please define a build platform."

#endif // JETSON_B01

/********************************************************************************
 * Exported objects
 ********************************************************************************/
#ifdef JETSON_B01

extern bool valid_image_rcvd;
extern videoSource *input;
extern uchar3 *image;
extern float input_video_width;
extern float input_video_height;

#elif _WIN32

extern bool valid_image_rcvd;
extern cv::Mat image;
extern float input_video_width;
extern float input_video_height;

#else

#error "Please define a build platform."

#endif // JETSON_B01

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
