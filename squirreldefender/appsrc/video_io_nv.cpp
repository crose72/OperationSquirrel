#ifdef ENABLE_CV
#ifdef BLD_JETSON_B01

/********************************************************************************
 * @file    video_io_nv.cpp
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Configure and start video streams using Jetson Inference library.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include <string>
#include <fstream>

#include "common_inc.h"
#include "video_io_nv.h"
#include "target_detection.h"
#include "jetson-utils/videoSource.h"
#include "jetson-utils/videoOutput.h"
#include "jetson-inference/detectNet.h"
#include "jetson-inference/objectTracker.h"
#include <jetson-inference/objectTrackerIOU.h>

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/
bool g_cam0_img_valid;
std::string base_path = "../data/";
bool display_output_created_nv;
bool file_output_created_nv;
videoSource *g_cam0_stream_nv;
videoOutput *vid_out_nv;
videoOutput *vid_out_disp_nv;
uchar3 *g_cam0_img_cpu;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/
const float g_cam0_img_width_px = (float)1280.0;
const float g_cam0_img_height_px = (float)720.0;
const float g_input_video_fps = (float)30.0;

/********************************************************************************
 * Function definitions
 ********************************************************************************/
bool video_output_file_reset(void);
bool create_input_video_stream(void);
bool create_output_vid_stream(void);
bool create_display_video_stream(void);
bool capture_image(void);
bool save_video(void);
bool display_video(void);
void calc_video_res(void);
void delete_input_video_stream(void);
void delete_video_file_stream(void);
void delete_video_display_stream(void);

/********************************************************************************
 * Function: file_exists
 * Description: Return true if file name already exists.
 ********************************************************************************/
bool file_exists(const std::string &name)
{
    std::ifstream f(name.c_str());
    return f.good();
}

/********************************************************************************
 * Function: generate_unique_file_name
 * Description: Generate a unique file name by incrementing the name by 1.
 ********************************************************************************/
std::string generate_unique_file_name(const std::string &base_name, const std::string &extension)
{
    std::string file_name = base_path + base_name + extension;
    int index = 1;

    while (file_exists(file_name))
    {
        file_name = base_path + base_name + "_" + std::to_string(index) + extension;
        index++;
    }

    return file_name;
}

/********************************************************************************
 * Function: create_input_video_stream
 * Description: Create an input video stream from the attached cameras.
 ********************************************************************************/
bool create_input_video_stream(void)
{
    videoOptions options;

    options.resource = URI("csi://0");
    options.resource.protocol = "csi";
    options.resource.location = "0";
    options.deviceType = videoOptions::DeviceType::DEVICE_CSI;
    options.ioType = videoOptions::IoType::INPUT;
    options.width = 1280; // 1280 for imx219-83
    options.height = 720; // 720 for imx219-83
    options.frameRate = 30;
    options.numBuffers = 4;
    options.zeroCopy = true;
    options.flipMethod = videoOptions::FlipMethod::FLIP_NONE; // if using IMX219-83 stereo camera
    // if using IMX219-160 stereo camera or H136 V1.3 and need to use option FLIP_ROTATE_180 then
    // you must compile jetson inference with cmake-DENABLE_NVMM=off

    g_cam0_stream_nv = videoSource::Create(options);

    if (!g_cam0_stream_nv)
    {
        LogError("detectnet:  failed to create input stream\n");
        return false;
    }

    return true;
}

/********************************************************************************
 * Function: create_output_vid_stream
 * Description: Create an output video stream to save to a file
 ********************************************************************************/
bool create_output_vid_stream(void)
{
    base_path = "file:///home/crose72/workspaces/os-dev/operationsquirrel/squirreldefender/data/";
    std::string base_name = "output";
    std::string extension = ".mp4";
    std::string file_name = generate_unique_file_name(base_name, extension);

    videoOptions options;

    options.resource = base_path + file_name;
    options.resource.protocol = "file";
    options.resource.location = file_name;
    options.deviceType = videoOptions::DeviceType::DEVICE_FILE;
    options.ioType = videoOptions::IoType::OUTPUT;
    options.codec = videoOptions::Codec::CODEC_H264;
    options.codecType = videoOptions::CodecType::CODEC_OMX;
    options.width = 1280;
    options.height = 720;
    options.frameRate = 30;
    options.zeroCopy = true;

    vid_out_nv = videoOutput::Create(options);

    if (!vid_out_nv)
    {
        LogError("detectnet:  failed to create vid_out_nv stream\n");
        file_output_created_nv = false;
        return false;
    }

    file_output_created_nv = true;

    return true;
}

/********************************************************************************
 * Function: create_display_video_stream
 * Description: Create an vid_out_disp_nv video stream from the input video stream
 *			   for displaying and passing to other software components.
 ********************************************************************************/
bool create_display_video_stream(void)
{
    videoOptions options;

    options.resource = "display://0"; // Specify the display URI
    options.resource.protocol = "display";
    options.resource.location = "0";
    options.deviceType = videoOptions::DeviceType::DEVICE_DISPLAY;
    options.ioType = videoOptions::IoType::OUTPUT;
    options.width = 1920;  // 1280 for imx219-83
    options.height = 1080; // 720 for imx219-83
    options.frameRate = 30;
    options.numBuffers = 4;
    options.zeroCopy = true;

    vid_out_disp_nv = videoOutput::Create(options);

    if (!vid_out_disp_nv)
    {
        LogError("detectnet:  failed to create vid_out_disp_nv stream\n");
        display_output_created_nv = false;
        return false;
    }

    display_output_created_nv = true;

    return true;
}

/********************************************************************************
 * Function: capture_image
 * Description: Capture an image from the input video stream.
 ********************************************************************************/
bool capture_image(void)
{
    int status = 0;

    if (!g_cam0_stream_nv->Capture(&g_cam0_img_cpu, &status))
    {
        if (status != videoSource::TIMEOUT)
        {
            return false;
        }
    }

    // Checking for valid g_cam0_img_cpu, Capture may return true while g_cam0_img_cpu may still be NULL
    if (g_cam0_img_cpu == NULL)
    {
        g_cam0_img_valid = false;
        return false; // Return false if the g_cam0_img_cpu is not valid
    }

    g_cam0_img_valid = true;

    return true;
}

/********************************************************************************
 * Function: save_video
 * Description: Save video to a file
 ********************************************************************************/
bool save_video(void)
{
    // Write current frame into the MP4 output file
    if (vid_out_nv != NULL)
    {
        vid_out_nv->Render(g_cam0_img_cpu, g_cam0_stream_nv->GetWidth(), g_cam0_stream_nv->GetHeight());

        // check if the user quit
        if (!vid_out_nv->IsStreaming())
        {
            return false;
        }
    }

    return true;
}

/********************************************************************************
 * Function: display_video
 * Description: Display the g_cam0_img_cpu on the screen.
 ********************************************************************************/
bool display_video(void)
{
    // render vid_out_disp_nv to the display
    if (vid_out_disp_nv != NULL)
    {
        vid_out_disp_nv->Render(g_cam0_img_cpu, g_cam0_stream_nv->GetWidth(), g_cam0_stream_nv->GetHeight());

#ifdef DEBUG_BUILD

        // update the status bar
        char str[256];
        sprintf(str, "TensorRT %i.%i.%i | %s | Network %.0f FPS", NV_TENSORRT_MAJOR, NV_TENSORRT_MINOR, NV_TENSORRT_PATCH, precisionTypeToStr(g_det_nv_net->GetPrecision()), g_det_nv_net->GetNetworkFPS());
        vid_out_disp_nv->SetStatus(str);

#endif // DEBUG_BUILD

        // check if the user quit
        if (!vid_out_disp_nv->IsStreaming())
        {
            return false;
        }
    }

    return true;
}

/********************************************************************************
 * Function: delete_input_video_stream
 * Description: Delete the input to stop using resources.
 ********************************************************************************/
void delete_input_video_stream(void)
{
    SAFE_DELETE(g_cam0_stream_nv);
}

/********************************************************************************
 * Function: delete_video_file_stream
 * Description: Delete the output to the video file to stop using resources.
 ********************************************************************************/
void delete_video_file_stream(void)
{
    SAFE_DELETE(vid_out_nv);
}

/********************************************************************************
 * Function: delete_video_display_stream
 * Description: Delete the display to stop using resources.
 ********************************************************************************/
void delete_video_display_stream(void)
{
    SAFE_DELETE(vid_out_disp_nv);
}

/********************************************************************************
 * Function: video_output_file_reset
 * Description: Code to reset video streams.
 ********************************************************************************/
bool video_output_file_reset(void)
{
    delete_video_file_stream();

    if (!create_output_vid_stream())
    {
        return false;
    }

    return true;
}

/********************************************************************************
 * Function: Video
 * Description: Constructor of the Video class.
 ********************************************************************************/
VideoNV::VideoNV(void) {}

/********************************************************************************
 * Function: Video
 * Description: Constructor of the Video class.
 ********************************************************************************/
VideoNV::~VideoNV(void) {}

/********************************************************************************
 * Function: init
 * Description: Code to initialize video streams to run onces at the start of the
 *              program.
 ********************************************************************************/
bool VideoNV::init(void)
{

    g_cam0_img_valid = false;
    g_cam0_img_cpu = NULL;

    if (!create_input_video_stream() ||
        !create_output_vid_stream())
    {
        return false;
    }

#ifdef DEBUG_BUILD

    // note: put in debug build
    if (!create_display_video_stream())
    {
        return false;
    }

#endif // DEBUG_BUILD

    return true;
}

/********************************************************************************
 * Function: in_loop
 * Description: Main video processing loop.
 ********************************************************************************/
void VideoNV::in_loop(void)
{
    capture_image();
}

/********************************************************************************
 * Function: out_loop
 * Description: Code needed to run each loop to  provide the video output.
 ********************************************************************************/
void VideoNV::out_loop(void)
{

#ifdef DEBUG_BUILD

    // note: put in debug build
    if (display_output_created_nv)
    {
        display_video();
    }

#endif // DEBUG_BUILD

    if (file_output_created_nv)
    {
        save_video();
    }
}

/********************************************************************************
 * Function: shutdown
 * Description: Shutdown detection network
 ********************************************************************************/
void VideoNV::shutdown(void)
{
    LogVerbose("video:  shutting down...\n");
    delete_input_video_stream();
    delete_video_file_stream();

    // note: put in debug build
    delete_video_display_stream();

#ifdef DEBUG_BUILD

#endif // DEBUG_BUILD

    LogVerbose("video:  shutdown complete.\n");
}

#endif // BLD_JETSON_B01
#endif // ENABLE_CV
