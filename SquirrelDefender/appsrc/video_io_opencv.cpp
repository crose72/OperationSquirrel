#ifdef ENABLE_CV
#if defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WIN)

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
bool g_valid_image_rcvd;
bool file_stream_created;

std::string base_path = "../data/";

cv::Mat g_image;
cv::VideoCapture cap;
cv::VideoWriter video_writer;
std::string gst_pipeline;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/
float g_input_video_width;
float g_input_video_height;
float g_input_video_fps;

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
    #ifdef BLD_JETSON_ORIN_NANO

    //gst_pipeline = std::format("nvarguscamerasrc ! video/x-raw(memory:NVMM), "
    //    "width={}, height={}, framerate={}/1 ! nvvidconv ! "
    //    "nvvidconv flip-method=2 ! "
    //    "video/x-raw, format=(string)BGRx ! videoconvert ! "
    //    "video/x-raw, format=(string)BGR ! appsink drop=true sync=false",
    //    g_input_video_width, g_input_video_height, g_input_video_fps);
    std::ostringstream ss;
    ss << "nvarguscamerasrc ! video/x-raw(memory:NVMM), "
       << "width=" << g_input_video_width << ", height=" << g_input_video_height 
       << ", framerate=" << g_input_video_fps << "/1 ! nvvidconv ! "
       << "nvvidconv flip-method=2 ! "
       << "video/x-raw, format=(string)BGRx ! videoconvert ! "
       << "video/x-raw, format=(string)BGR ! appsink drop=true sync=false";
    gst_pipeline = ss.str();

    cap.open(gst_pipeline, cv::CAP_GSTREAMER);

    #elif defined(BLD_WIN)

    cap.open(0, cv::CAP_DSHOW);  // Open default webcam
    cap.set(cv::CAP_PROP_FRAME_WIDTH, g_input_video_width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, g_input_video_height);

    #else

    #error "Please define a build platform."

    #endif

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
    //std::string base_path = "file:///home/crose72/Documents/GitHub/OperationSquirrel/SquirrelDefender/data/";
    #ifdef BLD_JETSON_ORIN_NANO
    std::string base_path = "file:///workspace/OperationSquirrel/SquirrelDefender/data/";
    #else
    std::string base_path = "file:///home/shaun/workspaces/os-dev/OperationSquirrel/SquirrelDefender/data/";
    #endif
    std::string base_name = "output";
    std::string extension = ".mp4";
    std::string file_name = generate_unique_file_name(base_name, extension);
    
    cv::Size frame_size(g_input_video_width, g_input_video_height); // 1280x720
    int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v'); // Codec for MP4 format
    video_writer.open(file_name, fourcc, g_input_video_fps, frame_size);

    if (!video_writer.isOpened()) {
        std::cout << "detectnet: failed to create output_vid_file stream\n" << std::endl;
        file_stream_created = false;
        return false;
    }

    file_stream_created = true;
    
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
        g_valid_image_rcvd = false;
        return false;
    }

    g_valid_image_rcvd = true;

    return true;
}

/********************************************************************************
 * Function: save_video
 * Description: Save video to a file
 ********************************************************************************/
bool save_video(void)
{
    // write video file 
    if (video_writer.isOpened())
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
 * Function: calc_video_res
 * Description: Delete the input to stop using resources.
 ********************************************************************************/
void calc_video_res(void)
{
    g_input_video_width = 1280.0;
    g_input_video_height = 720.0;
    g_input_video_fps = 30;
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

    calc_video_res(); // populate g_input_video_width, height 

    if (!create_input_video_stream() ||
        !create_output_vid_stream())
    {
        return false;
    }

#ifdef DEBUG_BUILD
    


#endif // DEBUG_BUILD


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

#ifdef DEBUG_BUILD

    // note: put in debug build
    display_video();

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
void VideoCV::shutdown(void)
{
    Print::cpp_cout("video:  shutting down...");
    Print::cpp_cout("video:  shutdown complete.\n");
}

#endif // defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WIN)
#endif // ENABLE_CV
