#ifdef ENABLE_CV
#ifdef BLD_WIN

/********************************************************************************
 * @file    video_win_IO.cpp
 * @author  Cameron Rose
 * @date    5/22/2024
 * @brief   Configure and start video streams for other software components.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "video_win_io.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/
bool valid_image_rcvd;

std::string base_path = "../data/";

cv::Mat image;
cv::VideoCapture cap;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/
float input_video_width;
float input_video_height;

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
    cap.open(0, cv::CAP_DSHOW);  // Open default webcam
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

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
    cap >> image;  // Capture image from the webcam

    if (image.empty()) {
        std::cout << "Error: Could not capture image" << std::endl;
        return false;
    }

    valid_image_rcvd = true;

    return true;
}

/********************************************************************************
 * Function: display_video
 * Description: Display the image on the screen.
 ********************************************************************************/
bool display_video(void)
{
    cv::imshow("MyVid", image);
    cv::waitKey(1);

    return true;
}

/********************************************************************************
 * Function: calc_video_res
 * Description: Delete the input to stop using resources.
 ********************************************************************************/
void calc_video_res(void)
{
    input_video_width = 1280.0;
    input_video_height = 720.0;
}

/********************************************************************************
 * Function: Video
 * Description: Constructor of the Video class.
 ********************************************************************************/
VideoWin::VideoWin(void) {}

/********************************************************************************
 * Function: Video
 * Description: Constructor of the Video class.
 ********************************************************************************/
VideoWin::~VideoWin(void) {}

/********************************************************************************
 * Function: init
 * Description: Code to initialize video streams to run onces at the start of the
 *              program.
 ********************************************************************************/
bool VideoWin::init(void)
{
    valid_image_rcvd = false;
    image = NULL;

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
void VideoWin::in_loop(void)
{
    capture_image();
}

/********************************************************************************
 * Function: out_loop
 * Description: Code needed to run each loop to  provide the video output.
 ********************************************************************************/
void VideoWin::out_loop(void)
{
    display_video();
}

/********************************************************************************
 * Function: shutdown
 * Description: Shutdown detection network
 ********************************************************************************/
void VideoWin::shutdown(void)
{
    PrintPass::cpp_cout("video:  shutting down...");
    PrintPass::cpp_cout("video:  shutdown complete.\n");
}

#endif // BLD_WIN
#endif // ENABLE_CV
