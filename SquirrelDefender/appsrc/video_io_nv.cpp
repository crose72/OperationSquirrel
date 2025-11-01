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
#include "common_inc.h"
#include "video_io_nv.h"
#include "detect_target.h"
#include <string>
#include <fstream>
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
bool g_cam0_valid_image_rcvd;
std::string base_path = "../data/";
bool display_stream_created;
bool file_stream_created;
videoSource *g_input;
videoOutput *output_vid_file;
videoOutput *output_vid_disp;
uchar3 *g_cam0_image;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/
const float g_cam0_video_width = 1280.0;
const float g_cam0_video_height = 720.0;
const float g_input_video_fps = 30;

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

    g_input = videoSource::Create(options);

    if (!g_input)
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
    std::string base_path = "file:///home/crose72/workspaces/os-dev/OperationSquirrel/SquirrelDefender/data/";
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

    output_vid_file = videoOutput::Create(options);

    if (!output_vid_file)
    {
        LogError("detectnet:  failed to create output_vid_file stream\n");
        file_stream_created = false;
        return false;
    }

    file_stream_created = true;

    return true;
}

/********************************************************************************
 * Function: create_display_video_stream
 * Description: Create an output_vid_disp video stream from the input video stream
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

    output_vid_disp = videoOutput::Create(options);

    if (!output_vid_disp)
    {
        LogError("detectnet:  failed to create output_vid_disp stream\n");
        display_stream_created = false;
        return false;
    }

    display_stream_created = true;

    return true;
}

/********************************************************************************
 * Function: capture_image
 * Description: Capture an image from the input video stream.
 ********************************************************************************/
bool capture_image(void)
{
    int status = 0;

    if (!g_input->Capture(&g_cam0_image, &status))
    {
        if (status != videoSource::TIMEOUT)
        {
            return false;
        }
    }

    // Checking for valid g_cam0_image, Capture may return true while g_cam0_image may still be NULL
    if (g_cam0_image == NULL)
    {
        g_cam0_valid_image_rcvd = false;
        return false; // Return false if the g_cam0_image is not valid
    }

    g_cam0_valid_image_rcvd = true;

    return true;
}

/********************************************************************************
 * Function: save_video
 * Description: Save video to a file
 ********************************************************************************/
bool save_video(void)
{
    // render output_vid_disp to the display
    if (output_vid_file != NULL)
    {
        output_vid_file->Render(g_cam0_image, g_input->GetWidth(), g_input->GetHeight());

        // check if the user quit
        if (!output_vid_file->IsStreaming())
        {
            return false;
        }
    }

    return true;
}

/********************************************************************************
 * Function: display_video
 * Description: Display the g_cam0_image on the screen.
 ********************************************************************************/
bool display_video(void)
{
    // render output_vid_disp to the display
    if (output_vid_disp != NULL)
    {
        output_vid_disp->Render(g_cam0_image, g_input->GetWidth(), g_input->GetHeight());

#ifdef DEBUG_BUILD

        // update the status bar
        char str[256];
        sprintf(str, "TensorRT %i.%i.%i | %s | Network %.0f FPS", NV_TENSORRT_MAJOR, NV_TENSORRT_MINOR, NV_TENSORRT_PATCH, precisionTypeToStr(g_net->GetPrecision()), g_net->GetNetworkFPS());
        output_vid_disp->SetStatus(str);

#endif // DEBUG_BUILD

        // check if the user quit
        if (!output_vid_disp->IsStreaming())
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
    SAFE_DELETE(g_input);
}

/********************************************************************************
 * Function: delete_video_file_stream
 * Description: Delete the output to the video file to stop using resources.
 ********************************************************************************/
void delete_video_file_stream(void)
{
    SAFE_DELETE(output_vid_file);
}

/********************************************************************************
 * Function: delete_video_display_stream
 * Description: Delete the display to stop using resources.
 ********************************************************************************/
void delete_video_display_stream(void)
{
    SAFE_DELETE(output_vid_disp);
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

    g_cam0_valid_image_rcvd = false;
    g_cam0_image = NULL;

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
    if (display_stream_created)
    {
        display_video();
    }

#endif // DEBUG_BUILD

    if (file_stream_created)
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
