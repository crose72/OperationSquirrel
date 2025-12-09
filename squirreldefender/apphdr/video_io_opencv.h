#ifdef ENABLE_CV
#if defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WIN) || defined(BLD_WSL)

/********************************************************************************
 * @file    video_io_opencv.h
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Video capture and recording interface using OpenCV and GStreamer.
 *          Supports Jetson Orin Nano (CSI via GStreamer), Windows webcams,
 *          and WSL2 virtual capture. Provides CPU and GPU frame access for
 *          downstream detection and tracking modules.
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
extern bool g_cam0_img_valid;
extern cv::Mat g_cam0_img_cpu;
extern float g_cam0_img_width_px;
extern float g_cam0_img_height_px;
extern bool g_video_end;
extern uint32_t g_cam0_frame_id;
extern cv::cuda::GpuMat g_cam0_img_gpu;
extern float g_cam0_fov_deg;
extern float g_cam0_img_width_cx;
extern float g_cam0_img_height_cy;
extern float g_cam0_fov_rad;
extern float g_cam0_fov_rad_half;
extern float g_cam0_tilt_deg;
extern float g_cam0_tilt_rad;

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
