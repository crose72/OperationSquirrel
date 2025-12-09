#ifdef ENABLE_CV

/********************************************************************************
 * @file    video_io.h
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Platform-selection layer for video input/output. Routes video I/O
 *          to the Jetson Nano B01 (VideoNV) or to the OpenCV/GStreamer path
 *          (VideoCV) for all other supported platforms.
 ********************************************************************************/
#ifndef VIDEO_IO_H
#define VIDEO_IO_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#ifdef BLD_JETSON_B01

#include "video_io_nv.h"

#elif defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WIN) || defined(BLD_WSL)

#include "video_io_opencv.h"

#else

#error "Please define a build platform."

#endif // BLD_JETSON_B01

/********************************************************************************
 * Imported objects
 ********************************************************************************/

/********************************************************************************
 * Exported objects
 ********************************************************************************/

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
