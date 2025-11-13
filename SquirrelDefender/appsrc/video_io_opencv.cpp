#ifdef ENABLE_CV
#if defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WIN) || defined(BLD_WSL)

/********************************************************************************
 * @file    video_io_cv.cpp
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Configure and start video streams on jetson orin using opencv and
 *          gstreamer.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "video_io_opencv.h"
#include "time_calc.h"
#include <opencv2/cudaimgproc.hpp>
#include <spdlog/spdlog.h>
#include <sstream>
#include <filesystem>
#include <fstream>
#include <cmath>

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/
std::string base_path = "../data/";
cv::Mat g_cam0_image;
cv::cuda::GpuMat g_cam0_image_gpu;
cv::VideoCapture cam0_capture;
bool g_cam0_valid_image_rcvd;
uint32_t g_cam0_frame_id;
float g_cam0_video_width_center;
float g_cam0_video_height_center;
float g_cam0_fov_rad;
float g_cam0_fov_rad_half;
bool g_end_of_video;
float cam0_video_out_fps_actual;
bool file_stream_created;
cv::VideoWriter video_writer;
cv::Mat image_overlay;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/
float g_cam0_video_width = (float)1640.0;
float g_cam0_video_height = (float)1232.0;
float g_camera_fov = (float)105.0; // 1.832595 rad for 105 fov // 1.4486 rad for 83 fov
float cam0_video_out_fps = (float)21.0;
float g_cam0_tilt_down_angle = (float)0.0;
float g_cam0_tilt_down_angle_rad = (float)0.0;

/********************************************************************************
 * Function definitions
 ********************************************************************************/
void get_video_io_params(void);
bool create_video_io_streams(void);
bool capture_image(void);
bool save_video(void);
bool display_video(void);
void delete_input_video_stream(void);
void delete_output_video_stream(void);

// File operations
bool file_exists(const std::string &name);
std::string generate_unique_file_name(const std::string &base_name, const std::string &extension);

// Text overlay functions
void overlay_text(const cv::Mat &image, const std::string &label, const std::string &value, cv::Point position, double font_scale, cv::Scalar color, int thickness);
void video_mods(void);

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

#include <string>
#include <sstream>
#include "param_reader.h" // Your existing ParamReader class

/********************************************************************************
 * Function: get_video_io_params
 * Description: Video input and output paramters.
 ********************************************************************************/
void get_video_io_params(void)
{
    ParamReader cfg("../params.json");

    g_cam0_video_width = cfg.get_float_param("Camera_Control_Params", "Input_Width");
    g_cam0_video_height = cfg.get_float_param("Camera_Control_Params", "Input_Height");
    cam0_video_out_fps = cfg.get_float_param("Camera_Control_Params", "Output_Framerate");
    g_camera_fov = cfg.get_float_param("Camera_Control_Params", "Output_Framerate");
    g_cam0_tilt_down_angle = cfg.get_float_param("Camera_Control_Params", "CAM0_Tilt_Down_Angle");
    g_cam0_tilt_down_angle_rad = g_cam0_tilt_down_angle * M_PI / (float)180.0;
    g_cam0_fov_rad = g_camera_fov * M_PI / (float)180.0;
    g_cam0_fov_rad_half = g_cam0_fov_rad * (float)0.5;
}

/********************************************************************************
 * Function: create_gstreamer_pipelines
 * Description: Create the gstreamer pipelines for video input and output.
 ********************************************************************************/
void create_gstreamer_pipelines(std::string &capture_pipeline,
                                std::string &writer_pipeline,
                                const std::string &output_path)
{
    ParamReader cfg("../params.json");

    /**************** CAMERA PARAMETERS ****************/
    int sensor_mode = cfg.get_int_param("Camera_Params", "Sensor_Mode");
    int wbmode = cfg.get_int_param("Camera_Params", "WBMode");
    int aeantibanding = cfg.get_int_param("Camera_Params", "AEAntiBanding");
    int tnr_mode = cfg.get_int_param("Camera_Params", "TNR_Mode");
    int ee_mode = cfg.get_int_param("Camera_Params", "EE_Mode");
    int exposure_min = cfg.get_int_param("Camera_Params", "Exposure_Min");
    int exposure_max = cfg.get_int_param("Camera_Params", "Exposure_Max");
    float gain_min = cfg.get_float_param("Camera_Params", "Gain_Min");
    float gain_max = cfg.get_float_param("Camera_Params", "Gain_Max");
    float isp_dgain_min = cfg.get_float_param("Camera_Params", "ISP_DGain_Min");
    float isp_dgain_max = cfg.get_float_param("Camera_Params", "ISP_DGain_Max");

    int input_width = cfg.get_int_param("Camera_Params", "Capture_Width");
    int input_height = cfg.get_int_param("Camera_Params", "Capture_Height");
    std::string input_format = cfg.get_string_param("Camera_Params", "Input_Format");
    std::string input_rate = cfg.get_string_param("Camera_Params", "Input_Framerate");

    int output_width = cfg.get_int_param("Camera_Params", "Output_Width");
    int output_height = cfg.get_int_param("Camera_Params", "Output_Height");
    std::string output_format = cfg.get_string_param("Camera_Params", "Output_Format");

    /**************** ENCODER PARAMETERS ****************/
    int bitrate = cfg.get_int_param("Encoder_Params", "Bitrate");
    std::string speed_preset = cfg.get_string_param("Encoder_Params", "Speed_Preset");
    std::string tune = cfg.get_string_param("Encoder_Params", "Tune");
    int key_int_max = cfg.get_int_param("Encoder_Params", "KeyIntMax");
    int threads = cfg.get_int_param("Encoder_Params", "Threads");
    bool faststart = cfg.get_bool_param("Encoder_Params", "Mux_FastStart");

    /**************** BUILD CAPTURE PIPELINE ****************/
    std::ostringstream ss_cap;
    ss_cap << "nvarguscamerasrc sensor-mode=" << sensor_mode
           << " wbmode=" << wbmode
           << " aeantibanding=" << aeantibanding
           << " tnr-mode=" << tnr_mode
           << " ee-mode=" << ee_mode
           << " exposuretimerange=\"" << exposure_min << " " << exposure_max << "\""
           << " gainrange=\"" << gain_min << " " << gain_max << "\""
           << " ispdigitalgainrange=\"" << isp_dgain_min << " " << isp_dgain_max << "\" ! "
           << "video/x-raw(memory:NVMM),format=" << input_format
           << ",width=" << input_width << ",height=" << input_height
           << ",framerate=" << input_rate << " ! "
           << "nvvidconv ! video/x-raw,format=BGRx,width=" << output_width
           << ",height=" << output_height << " ! "
           << "videoconvert ! video/x-raw,format=" << output_format
           << ",width=" << output_width << ",height=" << output_height << " ! "
           << "appsink drop=true max-buffers=1 sync=false";

    capture_pipeline = ss_cap.str();

    /**************** BUILD OUTPUT PIPELINE ****************/
    std::ostringstream ss_out;
    ss_out << "appsrc format=time is-live=true do-timestamp=true "
           << "caps=video/x-raw,format=" << output_format
           << ",width=" << output_width << ",height=" << output_height
           << ",framerate=" << input_rate << " ! "
           << "videoconvert ! "
           << "x264enc bitrate=" << bitrate
           << " speed-preset=" << speed_preset
           << " tune=" << tune
           << " key-int-max=" << key_int_max
           << " threads=" << threads
           << " ! h264parse config-interval=1 ! "
           << "mp4mux faststart=" << (faststart ? "true" : "false")
           << " ! filesink location=" << output_path;

    writer_pipeline = ss_out.str();
}

/********************************************************************************
 * Function: create_video_io_streams
 * Description: Create an input video stream from the attached cameras.
 ********************************************************************************/
bool create_video_io_streams(void)
{
#if defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WSL)
    const std::string base_path = "/workspace/OperationSquirrel/SquirrelDefender/data/";
#else
    const std::string base_path = "./";
#endif

    // Make sure the folder exists
    std::error_code ec;
    std::filesystem::create_directories(base_path, ec);

    const std::string base_name = "output";
    const std::string extension = ".mp4";
    const std::string file_name = generate_unique_file_name(base_name, extension);
    const std::string full_path = base_path + file_name;
    std::string video_cap_pipeline;
    std::string video_out_pipeline;

    create_gstreamer_pipelines(video_cap_pipeline, video_out_pipeline, full_path);

    // Use live camera feed or use prerecorded video
    if (!g_use_video_playback)
    {
#if defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WSL)

        cam0_capture.open(video_cap_pipeline, cv::CAP_GSTREAMER);

#elif defined(BLD_WIN)

        cam0_capture.open(0, cv::CAP_DSHOW); // Open default webcam
        cam0_capture.set(cv::CAP_PROP_FRAME_WIDTH, g_cam0_video_width);
        cam0_capture.set(cv::CAP_PROP_FRAME_HEIGHT, g_cam0_video_height);

#else

#error "Please define a build platform."

#endif
    }
    else
    {
        // Use video playback - get input video's parameters
        cam0_capture.open(g_input_video_path);
        g_cam0_video_width = cam0_capture.get(cv::CAP_PROP_FRAME_WIDTH);
        g_cam0_video_height = cam0_capture.get(cv::CAP_PROP_FRAME_HEIGHT);
        cam0_video_out_fps = cam0_capture.get(cv::CAP_PROP_FPS);
    }

    // Check that camera opens
    if (!cam0_capture.isOpened())
    {
        spdlog::error("Error: Could not open camera");
        return false;
    }

    // 21 is max fps for mode 0 (full resolution)
    if (!video_writer.open(video_out_pipeline, cv::CAP_GSTREAMER, 0, cam0_video_out_fps, cv::Size(g_cam0_video_width, g_cam0_video_height), true))
    {
        spdlog::error("video_io_opencv: failed to open GStreamer VideoWriter\n");
        file_stream_created = false;
        return false;
    }

    spdlog::info("Recording to: " + full_path);
    file_stream_created = true;

    // Calculate new center if video playback has a different size
    g_cam0_video_width_center = g_cam0_video_width * (float)0.5;
    g_cam0_video_height_center = g_cam0_video_height * (float)0.5;

    return true;
}

/********************************************************************************
 * Function: capture_image
 * Description: Capture an image from the input video stream.
 ********************************************************************************/
bool capture_image(void)
{
    cam0_capture >> g_cam0_image; // Capture g_cam0_image from the camera

    if (g_cam0_image.empty())
    {
        g_cam0_valid_image_rcvd = false;

        if (g_use_video_playback)
        {
            spdlog::info("End of video playback");
            g_end_of_video = true;
            return false;
        }

        spdlog::error("Error: Could not capture image");
        return false;
    }

    g_cam0_valid_image_rcvd = true;
    cam0_video_out_fps_actual = ((g_dt > (float)0.000001) ? (1 / g_dt) : (float)0.0);

    g_cam0_image_gpu.upload(g_cam0_image);
    ++g_cam0_frame_id;

    return true;
}

/********************************************************************************
 * Function: overlay_text
 * Description: Write text of desired size, color, and location onto a video
 *              frame.
 ********************************************************************************/
void overlay_text(
    const cv::Mat &image,
    const std::string &label,
    const std::string &value,
    cv::Point position,
    double font_scale,
    cv::Scalar color,
    int thickness)
{
    std::ostringstream stream;
    stream << std::fixed << value;

    std::string text = label + ": " + stream.str();

    cv::putText(image, text, position,
                cv::FONT_HERSHEY_SIMPLEX, font_scale, color, thickness);
}

/********************************************************************************
 * Function: video_mods
 * Description: Modifications to the image before writing to the video file
 *              if desired.
 ********************************************************************************/
void video_mods(void)
{
    // Leave original timestamps
    if (g_use_video_playback)
    {
        return;
    }

    overlay_text(
        g_cam0_image,
        "t",
        std::to_string(g_app_elapsed_time),
        cv::Point(10, g_cam0_image.rows - 10), // bottom-left
        0.5,                                   // font scale
        cv::Scalar(255, 255, 255),             // white
        1);
    overlay_text(
        g_cam0_image,
        "fps",
        std::to_string(cam0_video_out_fps_actual),
        cv::Point(10, g_cam0_image.rows - 30), // bottom-left
        0.5,                                   // font scale
        cv::Scalar(255, 255, 255),             // white
        1);
    overlay_text(
        g_cam0_image,
        "f",
        std::to_string(g_cam0_frame_id),
        cv::Point(10, g_cam0_image.rows - 50), // bottom-left
        0.5,                                   // font scale
        cv::Scalar(255, 255, 255),             // white
        1);
}

/********************************************************************************
 * Function: save_video
 * Description: Save video to a file
 ********************************************************************************/
bool save_video(void)
{
    // write video file
    if (video_writer.isOpened() && g_cam0_valid_image_rcvd && file_stream_created)
    {
        video_writer.write(g_cam0_image);
        return true;
    }

    return false;
}

/********************************************************************************
 * Function: display_video
 * Description: Display the image on the screen.
 ********************************************************************************/
bool display_video(void)
{
    if (g_cam0_valid_image_rcvd)
    {
        cv::imshow("Image", g_cam0_image);
        cv::waitKey(1);

        return true;
    }

    return false;
}

/********************************************************************************
 * Function: delete_input_video_stream
 * Description: Delete the input to stop using resources.
 ********************************************************************************/
void delete_input_video_stream(void)
{
    cam0_capture.release();
}

/********************************************************************************
 * Function: delete_output_video_stream
 * Description: Delete the output to the video file to stop using resources.
 ********************************************************************************/
void delete_output_video_stream(void)
{
    video_writer.release();
}

/********************************************************************************
 * Function: Video
 * Description: Constructor of the Video class.
 ********************************************************************************/
VideoCV::VideoCV(void) {}

/********************************************************************************
 * Function: Video
 * Description: Constructor of the Video class.
 ********************************************************************************/
VideoCV::~VideoCV(void) {}

/********************************************************************************
 * Function: init
 * Description: Code to initialize video streams to run onces at the start of the
 *              program.
 ********************************************************************************/
bool VideoCV::init(void)
{
    get_video_io_params();

    g_cam0_valid_image_rcvd = false;
    g_cam0_image = NULL;
    g_end_of_video = false;
    cam0_video_out_fps_actual = (float)16.67;
    g_cam0_frame_id = (uint32_t)0;

    if (!create_video_io_streams())
    {
        return false;
    }

    return true;
}

/********************************************************************************
 * Function: in_loop
 * Description: Main video processing loop, ie: the input.
 ********************************************************************************/
void VideoCV::in_loop(void)
{
    capture_image();
}

/********************************************************************************
 * Function: out_loop
 * Description: Code needed to run each loop to provide the video output.
 ********************************************************************************/
void VideoCV::out_loop(void)
{
    video_mods();
    save_video();

    if (g_use_video_playback)
    {
        display_video();
    }
}

/********************************************************************************
 * Function: shutdown
 * Description: Shutdown detection network
 ********************************************************************************/
void VideoCV::shutdown(void)
{
    spdlog::info("video:  shutting down...");
    delete_input_video_stream();
    delete_output_video_stream();
    spdlog::info("video:  shutdown complete.\n");
}

#endif // defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WIN) || defined(BLD_WSL)
#endif // ENABLE_CV
