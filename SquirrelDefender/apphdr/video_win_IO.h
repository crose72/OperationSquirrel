#pragma once

#ifdef ENABLE_CV
#ifdef _WIN32

/********************************************************************************
 * @file    video_win_IO.h
 * @author  Cameron Rose
 * @date    6/7/2023
 ********************************************************************************/
#ifndef VIDEO_WIN_IO_H
#define VIDEO_WIN_IO_H

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
extern bool valid_image_rcvd;
extern cv::Mat image;
extern float input_video_width;
extern float input_video_height;

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

#endif // VIDEO_WIN_IO_H

#endif // _WIN32
#endif // ENABLE_CV
