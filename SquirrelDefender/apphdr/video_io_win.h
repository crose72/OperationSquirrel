#pragma once

#ifdef ENABLE_CV
#ifdef BLD_WIN

/********************************************************************************
 * @file    video_io_win.h
 * @author  Cameron Rose
 * @date    1/22/2025
 ********************************************************************************/
#ifndef VIDEO_IO_WIN_H
#define VIDEO_IO_WIN_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include <string>
#include <fstream>
#include <opencv2/opencv.hpp>

/********************************************************************************
 * Imported objects
 ********************************************************************************/

/********************************************************************************
 * Exported objects
 ********************************************************************************/
extern bool g_valid_image_rcvd;
extern cv::Mat g_image;
extern float g_input_video_width;
extern float g_input_video_height;

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
class VideoWin
{
public:
    VideoWin();
    ~VideoWin();

    static bool init(void);
    static void in_loop(void);
    static void out_loop(void);
    static void shutdown(void);

private:
};

#endif // VIDEO_IO_WIN_H

#endif // BLD_WIN
#endif // ENABLE_CV
