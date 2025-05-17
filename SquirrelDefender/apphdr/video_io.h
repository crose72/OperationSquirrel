#pragma once

#ifdef ENABLE_CV

/********************************************************************************
 * @file    video_io.h
 * @author  Cameron Rose
 * @date    1/22/2025
 ********************************************************************************/
#ifndef VIDEO_IO_H
#define VIDEO_IO_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include <string>
#include <fstream>

#ifdef BLD_JETSON_B01

#include "video_io_nv.h"

#elif defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WIN)

#include "video_io_opencv.h"

#else

#error "Please define a build platform."

#endif // BLD_JETSON_B01


/********************************************************************************
 * Imported objects
 ********************************************************************************/
#ifdef BLD_JETSON_B01

extern detectNet *g_net;

#elif defined(BLD_JETSON_ORIN_NANO)

/* No imported objects */

#elif defined(BLD_WIN)

/* No imported objects */

#else

#error "Please define a build platform."

#endif // BLD_JETSON_B01

/********************************************************************************
 * Exported objects
 ********************************************************************************/
#ifdef BLD_JETSON_B01

extern bool g_valid_image_rcvd;
extern videoSource *g_input;
extern uchar3 *g_image;
extern float g_input_video_width;
extern float g_input_video_height;

#elif defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WIN)

extern bool g_valid_image_rcvd;
extern cv::Mat g_image;
extern float g_input_video_width;
extern float g_input_video_height;
extern bool g_end_of_video;

#else

#error "Please define a build platform."

#endif // BLD_JETSON_B01

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
