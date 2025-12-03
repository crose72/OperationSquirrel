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

/********************************************************************************
 * Exported objects
 ********************************************************************************/
extern bool g_cam0_img_valid;
extern videoSource *g_input;
extern uchar3 *g_cam0_img_cpu;
extern float g_cam0_img_width_px;
extern float g_cam0_img_height_px;

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
