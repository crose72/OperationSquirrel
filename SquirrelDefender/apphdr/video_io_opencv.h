#pragma once

#ifdef ENABLE_CV
#if defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WIN) || defined(BLD_WSL)

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
#include <opencv2/opencv.hpp>

/********************************************************************************
 * Exported objects
 ********************************************************************************/
extern bool g_valid_image_rcvd;
extern cv::Mat g_image;
extern float g_input_video_width;
extern float g_input_video_height;
extern bool g_end_of_video;
extern uint32_t g_frame_id;
extern cv::cuda::GpuMat g_image_gpu;
extern float g_camera_fov;
extern float g_input_video_width_center;
extern float g_input_video_height_center;

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
