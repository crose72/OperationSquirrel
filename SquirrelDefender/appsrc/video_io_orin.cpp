#ifdef ENABLE_CV
#ifdef BLD_JETSON_ORIN_NANO

/********************************************************************************
 * @file    video_io_orin.cpp
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Configure and start video streams on jetson orin using opencv and
 *          gstreamer.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "video_io_orin.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/
bool g_valid_image_rcvd;

std::string base_path = "../data/";

cv::Mat g_image;
cv::VideoCapture cap;
std::string gst_pipeline;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/
float g_input_video_width;
float g_input_video_height;

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
    gst_pipeline = 
        "nvarguscamerasrc ! video/x-raw(memory:NVMM), "
        "width=1280, height=720, framerate=30/1 ! nvvidconv ! "
        "video/x-raw, format=(string)BGRx ! videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink drop=true sync=false";

    cap.open(gst_pipeline, cv::CAP_GSTREAMER);

    if (!cap.isOpened())
    {
        std::cout << "Error: Could not open camera" << std::endl;
        return false;
    }

    return true;
}

/********************************************************************************
 * Function: capture_image
 * Description: Capture an image from the input video stream.
 ********************************************************************************/
bool capture_image(void)
{
    cap >> g_image;  // Capture g_image from the webcam

    if (g_image.empty()) {
        std::cout << "Error: Could not capture image" << std::endl;
        return false;
    }

    g_valid_image_rcvd = true;

    return true;
}

/********************************************************************************
 * Function: display_video
 * Description: Display the image on the screen.
 ********************************************************************************/
bool display_video(void)
{
    if (g_valid_image_rcvd)
    {
        cv::imshow("MyVid", g_image);
        cv::waitKey(1);

        return true;
    }

    return false;
}

/********************************************************************************
 * Function: calc_video_res
 * Description: Delete the input to stop using resources.
 ********************************************************************************/
void calc_video_res(void)
{
    g_input_video_width = 1280.0;
    g_input_video_height = 720.0;
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

    if (!create_input_video_stream())
    {
        return false;
    }

    calc_video_res();

    return true;
}

/********************************************************************************
 * Function: in_loop
 * Description: Main video processing loop.
 ********************************************************************************/
void VideoCV::in_loop(void)
{
    capture_image();
}

/********************************************************************************
 * Function: out_loop
 * Description: Code needed to run each loop to  provide the video output.
 ********************************************************************************/
void VideoCV::out_loop(void)
{
    display_video();
}

/********************************************************************************
 * Function: shutdown
 * Description: Shutdown detection network
 ********************************************************************************/
void VideoCV::shutdown(void)
{
    Print::cpp_cout("video:  shutting down...");
    Print::cpp_cout("video:  shutdown complete.\n");
}

#endif // BLD_JETSON_ORIN_NANO
#endif // ENABLE_CV
