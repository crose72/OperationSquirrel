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
#include <sstream>
#include <filesystem>
#include "video_io_opencv.h"

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
cv::Mat g_image;
cv::cuda::GpuMat g_image_gpu;
cv::Mat image_overlay;
cv::VideoCapture cap;
cv::VideoWriter video_writer;
float video_fps_actual;
bool g_valid_image_rcvd;
bool g_end_of_video;
bool file_stream_created;
uint32_t g_frame_id;

/********************************************************************************
 * Calibration definitionscd
 ********************************************************************************/
const float g_input_video_width = (float)1640.0;
const float g_input_video_height = (float)1232.0;
const float video_out_target_fps = (float)21.0;
float g_camera_fov = (float)1.832595; // 1.832595 // for 105 fov // 1.4486 for 83 fov

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
void delete_input_video_stream(void);
void delete_video_file_stream(void);
void delete_video_display_stream(void);

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

/********************************************************************************
 * Function: create_input_video_stream
 * Description: Create an input video stream from the attached cameras.
 ********************************************************************************/
bool create_input_video_stream(void)
{
    // Use live camera feed or use prerecorded video
    if (!g_use_video_playback)
    {
#if defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WSL)

        std::ostringstream ss;

        ss << "nvarguscamerasrc sensor-mode=0 "
              " wbmode=1 aeantibanding=2 tnr-mode=0 ee-mode=0 "
              " exposuretimerange=\"8000000 33333333\" gainrange=\"1 10\" ispdigitalgainrange=\"1 1.5\" ! "
              "video/x-raw(memory:NVMM),format=NV12,width=3280,height=2464,framerate=21/1 ! "
              "nvvidconv ! video/x-raw,format=BGRx,width=1640,height=1232 ! "
              "videoconvert ! video/x-raw,format=BGR,width=1640,height=1232 ! "
              "appsink drop=true max-buffers=1 sync=false";

        std::string video_cap_pipeline = ss.str();

        cap.open(video_cap_pipeline, cv::CAP_GSTREAMER);

        video_cap_pipeline = ss.str();

        cap.open(video_cap_pipeline, cv::CAP_GSTREAMER);

#elif defined(BLD_WIN)

        cap.open(0, cv::CAP_DSHOW); // Open default webcam
        cap.set(cv::CAP_PROP_FRAME_WIDTH, g_input_video_width);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, g_input_video_height);

#else

#error "Please define a build platform."

#endif
    }
    else
    {
        cap.open(input_video_path);
    }

    if (!cap.isOpened())
    {
        std::cout << "Error: Could not open camera" << std::endl;
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

    std::ostringstream ss;

    // Your capture frames are BGR, 1640x1232 @ 21 fps
    ss << "appsrc format=time is-live=true do-timestamp=true "
       << "caps=video/x-raw,format=BGR,width=1640,height=1232,framerate=21/1 ! "
       << "videoconvert ! "
       << "x264enc bitrate=16000 speed-preset=veryfast tune=zerolatency key-int-max=21 threads=0 ! "
       << "h264parse config-interval=1 ! mp4mux faststart=true ! filesink location=" << full_path;

    const std::string video_out_pipeline = ss.str();

    // 21 is max fps for mode 0 (full resolution)
    if (!video_writer.open(video_out_pipeline, cv::CAP_GSTREAMER, 0, video_out_target_fps, cv::Size(g_input_video_width, g_input_video_height), true))
    {
        std::cerr << "video_io_opencv: failed to open GStreamer VideoWriter\n";
        file_stream_created = false;
        return false;
    }

    std::cout << "Recording to: " << full_path << std::endl;
    file_stream_created = true;

    return true;
}

/********************************************************************************
 * Function: capture_image
 * Description: Capture an image from the input video stream.
 ********************************************************************************/
bool capture_image(void)
{
    cap >> g_image; // Capture g_image from the camera

    if (g_image.empty())
    {
        g_valid_image_rcvd = false;

        if (g_use_video_playback)
        {
            std::cout << "End of video playback" << std::endl;
            g_end_of_video = true;
            return false;
        }

        std::cout << "Error: Could not capture image" << std::endl;
        return false;
    }

    g_valid_image_rcvd = true;
    video_fps_actual = ((g_dt > (float)0.000001) ? (1 / g_dt) : (float)0.0);

    g_image_gpu.upload(g_image);
    ++g_frame_id;

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
    overlay_text(
        g_image,
        "t",
        std::to_string(g_app_elapsed_time),
        cv::Point(10, g_image.rows - 10), // bottom-left
        0.5,                              // font scale
        cv::Scalar(255, 255, 255),        // white
        1);
    overlay_text(
        g_image,
        "fps",
        std::to_string(video_fps_actual),
        cv::Point(10, g_image.rows - 30), // bottom-left
        0.5,                              // font scale
        cv::Scalar(255, 255, 255),        // white
        1);
    overlay_text(
        g_image,
        "f",
        std::to_string(g_frame_id),
        cv::Point(10, g_image.rows - 50), // bottom-left
        0.5,                              // font scale
        cv::Scalar(255, 255, 255),        // white
        1);
}

/********************************************************************************
 * Function: save_video
 * Description: Save video to a file
 ********************************************************************************/
bool save_video(void)
{
    // write video file
    if (video_writer.isOpened() && g_valid_image_rcvd && file_stream_created)
    {
        video_writer.write(g_image);
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
    if (g_valid_image_rcvd)
    {
        cv::imshow("Image", g_image);
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
    cap.release();
}

/********************************************************************************
 * Function: delete_video_file_stream
 * Description: Delete the output to the video file to stop using resources.
 ********************************************************************************/
void delete_video_file_stream(void)
{
    video_writer.release();
}

/********************************************************************************
 * Function: delete_video_display_stream
 * Description: Delete the display to stop using resources.
 ********************************************************************************/
void delete_video_display_stream(void)
{
    return;
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

    g_valid_image_rcvd = false;
    g_image = NULL;
    g_end_of_video = false;
    video_fps_actual = (float)16.67;
    g_frame_id = (uint32_t)0;

    if (!create_input_video_stream() ||
        !create_output_vid_stream())
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
    display_video();
}

/********************************************************************************
 * Function: shutdown
 * Description: Shutdown detection network
 ********************************************************************************/
void VideoCV::shutdown(void)
{
    Print::cpp_cout("video:  shutting down...");
    delete_input_video_stream();
    delete_video_file_stream();
    Print::cpp_cout("video:  shutdown complete.\n");
}

#endif // defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WIN) || defined(BLD_WSL)
#endif // ENABLE_CV
