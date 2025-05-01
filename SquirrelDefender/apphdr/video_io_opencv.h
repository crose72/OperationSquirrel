#pragma once

#ifdef ENABLE_CV
#if defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WIN)

/********************************************************************************
 * @file    video_io_opencv.h
 * @author  Cameron Rose
 * @date    1/22/2025
 ********************************************************************************/
#ifndef VIDEO_IO_CV_H
#define VIDEO_IO_CV_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include <string>
#include <fstream>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

/********************************************************************************
 * Imported objects
 ********************************************************************************/
extern float g_app_elapsed_time;
extern uint8_t g_mav_veh_state;
extern float g_x_target_ekf;
extern float g_y_target_ekf;
extern int32_t g_mav_veh_rel_alt;
extern std::string input_video_path;
extern bool g_use_video_playback;

/********************************************************************************
 * Exported objects
 ********************************************************************************/
extern bool g_valid_image_rcvd;
extern cv::Mat g_image;
extern float g_input_video_width;
extern float g_input_video_height;
extern bool g_end_of_video;

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
class VideoCV
{
public:
    VideoCV();
    ~VideoCV();

    static bool init(void);
    static void in_loop(void);
    static void out_loop(void);
    static void shutdown(void);

private:
};

#endif // VIDEO_IO_CV_H

#endif // defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WIN)
#endif // ENABLE_CV
