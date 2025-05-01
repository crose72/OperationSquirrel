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
bool g_end_of_video;
bool file_stream_created;

std::string base_path = "../data/";

cv::Mat g_image;
cv::Mat image_overlay;
cv::VideoCapture cap;
cv::VideoWriter video_writer;
std::string gst_pipeline;

// Structure to hold text configuration for each text element
struct TextOverlay {
    // Corner constants
    static const int TOP_LEFT = 0;
    static const int TOP_RIGHT = 1;
    static const int BOTTOM_LEFT = 2;
    static const int BOTTOM_RIGHT = 3;
    
    std::string text;
    cv::Point position;
    cv::Size text_size;
    cv::Rect bg_rect;
    cv::Scalar text_color;
    cv::Scalar bg_color;
    int font_face;
    double font_scale;
    int thickness;
    int corner;
    bool is_dynamic; // Flag for text that changes each frame
};

std::vector<TextOverlay> g_textOverlays;  // Vector to hold overlays

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
void delete_input_video_stream(void);
void delete_video_file_stream(void);
void delete_video_display_stream(void);

// File operations
bool file_exists(const std::string &name);
std::string generate_unique_file_name(const std::string &base_name, const std::string &extension);

// Text overlay functions
void init_text_overlay(const cv::Mat& referenceImage, const std::string& defaultText,
                       int corner, bool is_dynamic, cv::Scalar text_color, double font_scale);
void update_dynamic_text(int index, const std::string& newText);
void render_text_overlays(cv::Mat& image);
void video_overlay(void);

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

    if (!video_writer.isOpened())
    {
        std::cout << "video_io_opencv: failed to create output_vid_file stream\n" << std::endl;
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
    cap >> g_image;  // Capture g_image from the camera

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

    return true;
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
 * Function: init_text_overlay
 * Description: Initialize a text overlay config
 ********************************************************************************/
void init_text_overlay(const cv::Mat& referenceImage, const std::string& defaultText,
                       int corner, bool is_dynamic, cv::Scalar text_color, double font_scale)
{
    TextOverlay overlay;
    
    // Store parameters
    overlay.text = defaultText;
    overlay.is_dynamic = is_dynamic;
    overlay.text_color = text_color;
    overlay.bg_color = cv::Scalar(0, 0, 0); // Black background
    overlay.font_face = cv::FONT_HERSHEY_SIMPLEX;
    overlay.font_scale = font_scale;
    overlay.thickness = 1;
    
    // Calculate text size
    int baseline = 0;
    cv::Size text_size = cv::getTextSize(defaultText, overlay.font_face, 
                                        overlay.font_scale, overlay.thickness, &baseline);
    
    std::cout << std::to_string(referenceImage.cols) << std::endl;
    std::cout << std::to_string(referenceImage.rows) << std::endl;

    // Calculate fixed position based on corner
    switch(corner) 
    {
        case TextOverlay::TOP_LEFT:
            overlay.position = cv::Point(10, text_size.height + 10);
            break;
        case TextOverlay::TOP_RIGHT:
            overlay.position = cv::Point(referenceImage.cols - text_size.width - 10, 
                                       text_size.height + 10);
            break;
        case TextOverlay::BOTTOM_LEFT:
            overlay.position = cv::Point(10, referenceImage.rows - 10);
            break;
        case TextOverlay::BOTTOM_RIGHT:
            overlay.position = cv::Point(referenceImage.cols - text_size.width - 10, 
                                       referenceImage.rows - 10);
            break;
        default:
            overlay.position = cv::Point(10, text_size.height + 10);
    }
    
    // Pre-calculate background rectangle for static text
    if (!is_dynamic) 
    {
        overlay.bg_rect = cv::Rect(
            overlay.position.x - 5, 
            overlay.position.y - text_size.height - 5, 
            text_size.width + 10, 
            text_size.height + 10
        );
    } 
    else 
    {
        // For dynamic text, we'll use a default size that will be updated later
        // Store the maximum expected width to avoid text getting cut off
        int maxWidth = text_size.width * 1.5;  // Adjust this based on your needs
        overlay.bg_rect = cv::Rect(
            overlay.position.x - 5,
            overlay.position.y - text_size.height - 5,
            maxWidth + 10,
            text_size.height + 10
        );
    }
    
    // Add to global collection
    g_textOverlays.push_back(overlay);
}


/********************************************************************************
 * Function: update_dynamic_text
 * Description: Update the text for a dynamic overlay.
 ********************************************************************************/
void update_dynamic_text(int index, const std::string& newText) 
{
    if (index >= 0 && index < g_textOverlays.size() && g_textOverlays[index].is_dynamic) 
    {
        g_textOverlays[index].text = newText;
    }
}

/********************************************************************************
 * Function: render_text_overlays
 * Description: Draw all text overlays on the image.
 ********************************************************************************/
void render_text_overlays(cv::Mat& image) 
{
    for (const auto& overlay : g_textOverlays) 
    {
        // Draw background rectangle
        cv::rectangle(image, overlay.bg_rect, overlay.bg_color, -1);
        
        // Draw text
        cv::putText(image, overlay.text, overlay.position, 
                   overlay.font_face, overlay.font_scale, 
                   overlay.text_color, overlay.thickness);
    }
}

/********************************************************************************
 * Function: video_overlay
 * Description: Choose the text to overlay onto the video. 
 ********************************************************************************/
void video_overlay(void)
{
    image_overlay = g_image.clone();

    std::ostringstream ss;
    ss << "t: " << std::to_string(g_app_elapsed_time);
    std::string str_out_loop = ss.str();
    update_dynamic_text(0, str_out_loop);

    std::ostringstream ss2;
    ss2 << "mav_veh_state: " << std::to_string(g_mav_veh_state);
    str_out_loop = ss2.str();
    update_dynamic_text(1, str_out_loop);

    std::ostringstream ss3;
    ss3 << "x_target: " << std::to_string(g_x_target_ekf);
    str_out_loop = ss3.str();
    update_dynamic_text(2, str_out_loop);

    std::ostringstream ss4;
    ss4 << "alt: " << std::to_string(g_mav_veh_rel_alt);
    str_out_loop = ss4.str();
    update_dynamic_text(3, str_out_loop);

    render_text_overlays(image_overlay);
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
    g_input_video_width = 1280.0;
    g_input_video_height = 720.0;
    g_input_video_fps = 30;
    g_end_of_video = false;

    if (!create_input_video_stream() ||
        !create_output_vid_stream())
    {
        return false;
    }

    /*
    //init_text_overlay(g_image, "Hello world!", TextOverlay::TOP_LEFT);
    image_overlay = g_image.clone();
    init_text_overlay(image_overlay, "t: 4.206969", TextOverlay::BOTTOM_LEFT, true);
    init_text_overlay(image_overlay, "mav_veh_state: 1", TextOverlay::BOTTOM_RIGHT, true);
    init_text_overlay(image_overlay, "x_target: 4.20696969", TextOverlay::TOP_RIGHT, true);
    init_text_overlay(image_overlay, "alt: 0.420696969", TextOverlay::TOP_LEFT, true);
    */

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

#endif // defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WIN)
#endif // ENABLE_CV
