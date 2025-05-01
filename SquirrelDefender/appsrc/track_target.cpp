#ifdef ENABLE_CV

/********************************************************************************
 * @file    track_target.cpp
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Maintain bounding box around a detected target, even when object 
 *          detection fails.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "track_target.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/
bool target_tracked;
bool tracking;
bool initialized_cv_image;
bool initialized_tracker;
float target_detection_thresh;
float target_class;
bool target_valid_prv;
bool g_target_valid;
int g_target_detection_id;
int g_target_track_id;
float g_target_cntr_offset_x;
float g_target_cntr_offset_y;
float g_target_center_y;
float g_target_center_x;
float g_target_height;
float g_target_width;
float g_target_aspect;
float g_target_left;
float g_target_right;
float g_target_top;
float g_target_bottom;
float g_detection_class;
float g_target_detection_conf;

cv::cuda::GpuMat gpuImage;
cv::Mat image_cv_wrapped;
cv::Ptr<cv::TrackerCSRT> target_tracker;
cv::Rect target_bounding_box;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/
const float center_of_frame_width = 640.0f;
const float center_of_frame_height = 360.0f;

/********************************************************************************
 * Function definitions
 ********************************************************************************/
void get_tracking_params(void);
void identify_target(void);
void get_target_info(void);
void validate_target(void);
void track_target(void);
void update_target_info(void);
bool tracker_init(cv::Ptr<cv::TrackerCSRT> &tracker, cv::Mat &g_image, cv::Rect &bounding_box);
bool tracker_update(cv::Ptr<cv::TrackerCSRT> &tracker, cv::Mat &g_image, cv::Rect &bounding_box);

/********************************************************************************
 * Function: get_tracking_params
 * Description: Calibratable parameters for target tracking and identification.
 ********************************************************************************/
void get_tracking_params(void)
{
    ParamReader target_params("../params.json");

    target_detection_thresh = target_params.get_float_param("Detection_tracking", "Detect_Thresh");

#if defined(BLD_JETSON_B01)

    target_class = target_params.get_int_param("Detection_tracking", "Detect_Class_B01");

#elif defined(BLD_JETSON_ORIN_NANO)

    target_class = target_params.get_int_param("Detection_tracking", "Detect_Class_Orin");

#elif defined(BLD_WIN)

    target_class = target_params.get_int_param("Detection_tracking", "Detect_Class_Orin");

#else

#error "Please define build platform."

#endif
}

/********************************************************************************
 * Function: identify_target
 * Description: Determine which detected object to track.
 ********************************************************************************/
void identify_target(void)
{
    g_target_detection_id = -1;

#if defined(BLD_JETSON_B01)

    for (int n = 0; n < g_detection_count; ++n)
    {
        g_detection_class = g_detections[n].ClassID;
        g_target_detection_conf = g_detections[n].Confidence;
        /* A tracked object, classified as a person with some confidence level */
        if (g_detections[n].TrackID >= 0 && g_detection_class == target_class && g_target_detection_conf > target_detection_thresh)
        {
            g_target_detection_id = n;
            return;
        }
    }

#elif defined(BLD_JETSON_ORIN_NANO)

    for (int n = 0; n < g_yolo_detection_count; ++n)
    {   
        g_detection_class = g_yolo_detections[n].label;
        g_target_detection_conf = g_yolo_detections[n].probability;
        /* A tracked object, classified as a person with some confidence level */
        if (g_detection_class == target_class && g_yolo_detections[n].probability > target_detection_thresh)
        {
            g_target_detection_id = n;
            return;
        }
    }

#elif defined(BLD_WIN)

    for (int n = 0; n < g_yolo_detection_count; ++n)
    {
        g_detection_class = g_yolo_detections[n].ClassID;
        g_target_detection_conf = g_yolo_detections[n].Confidence;
        /* A tracked object, classified as a person with some confidence level */
        if (g_detection_class == target_class && g_target_detection_conf > target_detection_thresh)
        {
            g_target_detection_id = n;
            return;
        }
    }

#else

#error "Please define build platform."

#endif
}

/********************************************************************************
 * Function: get_target_info
 * Description: Obtain the important information about the target, such as the
 *              height and location of the center of the target in the frame.
 ********************************************************************************/
void get_target_info(void)
{
#if defined(BLD_JETSON_B01)

    if (g_target_detection_id >= 0)
    {
        g_target_height = g_detections[g_target_detection_id].Height();
        g_target_width = g_detections[g_target_detection_id].Width();
        g_target_track_id = g_detections[g_target_detection_id].TrackID;
        g_target_left = g_detections[g_target_detection_id].Left;
        g_target_right = g_detections[g_target_detection_id].Right;
        g_target_top = g_detections[g_target_detection_id].Top;
        g_target_bottom = g_detections[g_target_detection_id].Bottom;
        g_target_center_y = (g_target_left + g_target_right) / 2.0f;
        g_target_center_x = (g_target_bottom + g_target_top) / 2.0f;
        g_target_cntr_offset_y = g_target_center_y - center_of_frame_width;
        g_target_cntr_offset_x = g_target_center_x - center_of_frame_height;
        g_target_aspect = g_target_width / g_target_height;
    }

#elif defined(BLD_JETSON_ORIN_NANO)

    if (g_target_detection_id >= 0)
    {
        g_target_height = g_yolo_detections[g_target_detection_id].rect.height;
        g_target_width = g_yolo_detections[g_target_detection_id].rect.width;
        g_target_track_id = 0;
        g_target_left = g_yolo_detections[g_target_detection_id].rect.x;
        g_target_right = g_target_left + g_target_width;
        g_target_top = g_yolo_detections[g_target_detection_id].rect.y;
        g_target_bottom = g_target_top + g_target_height;
        g_target_center_y = (g_target_left + g_target_right) / 2.0f;
        g_target_center_x = (g_target_bottom + g_target_top) / 2.0f;
        g_target_cntr_offset_y = g_target_center_y - center_of_frame_width;
        g_target_cntr_offset_x = g_target_center_x - center_of_frame_height;
        g_target_aspect = g_target_width / g_target_height; 
    }

#elif defined(BLD_WIN)

    if (g_target_detection_id >= 0)
    {
        g_target_height = g_yolo_detections[g_target_detection_id].rect.height;
        g_target_width = g_yolo_detections[g_target_detection_id].rect.width;
        g_target_track_id = 0;
        g_target_left = g_yolo_detections[g_target_detection_id].rect.x;
        g_target_right = g_target_left + g_target_width;
        g_target_top = g_yolo_detections[g_target_detection_id].rect.y;
        g_target_bottom = g_target_top + g_target_height;
        g_target_center_y = (g_target_left + g_target_right) / 2.0f;
        g_target_center_x = (g_target_bottom + g_target_top) / 2.0f;
        g_target_cntr_offset_y = g_target_center_y - center_of_frame_width;
        g_target_cntr_offset_x = g_target_center_x - center_of_frame_height;
        g_target_aspect = g_target_width / g_target_height; 
    }

#else

#error "Please define a build platform."

#endif
}

/********************************************************************************
 * Function: validate_target
 * Description: Determine which detected object to track.
 ********************************************************************************/
void validate_target(void)
{
#if defined(BLD_JETSON_B01)

    /* Target detected, tracked, and has a size greater than 0.  Controls based on the target may be
       implimented. */
    if (g_target_detection_id >= 0 && g_target_track_id >= 0 && g_target_height > 1 && g_target_width > 1)
    {
        g_target_valid = true;
    }
    else
    {
        g_target_valid = false;
    }

#elif defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WIN)

    /* Target detected, tracked, and has a size greater than 0.  Controls based on the target may be
   implimented. */
    if (g_target_detection_id >= 0 && g_target_height > 1 && g_target_width > 1)
    {
        g_target_valid = true;
    }
    else
    {
        g_target_valid = false;
    }

#else

#error "Please define build platform."

#endif // defined(BLD_JETSON_B01) || defined(BLD_JETSON_ORIN_NANO)

    target_valid_prv = g_target_valid;
}

/* TODO: Make tracking work! */
#ifdef TRACKING_WORKS
/********************************************************************************
 * Function: tracker_init
 * Description: Initialize the tracker by resetting the bounding box to zeros.
 ********************************************************************************/
bool tracker_init(cv::Ptr<cv::TrackerCSRT> &tracker, cv::Mat &cv_image, cv::Rect &bounding_box)
{
    tracker->init(cv_image, bounding_box);

    return true;
}

/********************************************************************************
 * Function: tracker_update
 * Description: Update the bounding box around the tracked target.
 ********************************************************************************/
bool tracker_update(cv::Ptr<cv::TrackerCSRT> &tracker, cv::Mat &cv_image, cv::Rect &bounding_box)
{
    bool success;
    try {
        success = tracker->update(cv_image, bounding_box);
        if (!success) {
            std::cerr << "Tracking failed." << std::endl;
        }
    }
    catch (const cv::Exception& e) {
        std::cerr << "OpenCV error: " << e.what() << std::endl;
    }

    return success;
}
/********************************************************************************
 * Function: track_target
 * Description: Use the bounding box provided by the detection model as the
 *              basis for target_tracked the targeted object.
 ********************************************************************************/
void track_target(void)
{
#if defined(BLD_JETSON_B01)

/* Don't wrap the image from jetson inference until a valid image has been received.
   That way we know the memory has been allocaed and is ready. */
    if (g_valid_image_rcvd && !initialized_cv_image)
    {
        image_cv_wrapped = cv::Mat(g_input_video_height, g_input_video_width, CV_8UC3, g_image); // Directly wrap uchar3*
        initialized_cv_image = true;
    }
    else if (g_valid_image_rcvd && initialized_cv_image)
    {
        if (g_target_valid && !initialized_tracker)
        {
            target_bounding_box = cv::Rect(g_target_left, g_target_top, g_target_width, g_target_height);
            tracker_init(target_tracker, image_cv_wrapped, target_bounding_box);
            initialized_tracker = true;
        }

        if (initialized_tracker)
        {
            target_tracked = tracker_update(target_tracker, image_cv_wrapped, target_bounding_box);
        }

        if (target_tracked)
        {
            std::cout << "Tracking" << std::endl;
            cv::rectangle(image_cv_wrapped, target_bounding_box, cv::Scalar(255, 0, 0));

            tracking = true;
        }
        else
        {
            std::cout << "Not Tracking" << std::endl;
            initialized_tracker = false;
            tracking = false;
        }
    }

#elif defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WIN)

    /* Don't wrap the image from jetson inference until a valid image has been received.
    That way we know the memory has been allocaed and is ready. */
    if (g_valid_image_rcvd && !initialized_cv_image)
    {
        initialized_cv_image = true;
    }
    else if (g_valid_image_rcvd && initialized_cv_image)
    {
        if (g_target_valid && !initialized_tracker)
        {
            target_bounding_box = cv::Rect(g_target_left, g_target_top, g_target_width, g_target_height);
            tracker_init(target_tracker, g_image, target_bounding_box);
            initialized_tracker = true;
        }
        
        if (initialized_tracker)
        {
            target_tracked = tracker_update(target_tracker, g_image, target_bounding_box);
            std::cout << "target_tracked: " << target_tracked << std::endl;
        }
        if (target_tracked)
        {
            // Draw the target_tracked box
            cv::rectangle(g_image, target_bounding_box, cv::Scalar(255, 0, 0), 2, 1);
            tracking = true;
        }
        else
        {
            tracking = false;
        }
        
    }

#else

#error "Please define build platform."

#endif // defined(BLD_JETSON_B01) || defined(BLD_JETSON_ORIN_NANO)
}
#endif

/********************************************************************************
 * Function: update_target_info
 * Description: Obtain the important information about the target, such as the
 *              height and location of the center of the target in the frame.
 ********************************************************************************/
void update_target_info(void)
{
    g_target_height = target_bounding_box.height;
    g_target_width = target_bounding_box.width;
    g_target_left = target_bounding_box.x;
    g_target_right = g_target_left + g_target_width;
    g_target_top = target_bounding_box.y;
    g_target_bottom = g_target_top + g_target_height;
    g_target_center_y = (g_target_left + g_target_right) / 2.0f;
    g_target_center_x = (g_target_bottom + g_target_top) / 2.0f;
    g_target_cntr_offset_y = g_target_center_y - center_of_frame_width;
    g_target_cntr_offset_x = g_target_center_x - center_of_frame_height;
    g_target_aspect = g_target_width / g_target_height;
}

/********************************************************************************
 * Function: Track
 * Description: Track class constructor.
 ********************************************************************************/
Track::Track(void) {};

/********************************************************************************
 * Function: ~Track
 * Description: Track class destructor.
 ********************************************************************************/
Track::~Track(void) {};

/********************************************************************************
 * Function: init
 * Description: Initialize all track target variables.  Run once at the start
 *              of the program.
 ********************************************************************************/
bool Track::init(void)
{
    g_target_valid = false;
    target_valid_prv = false;
    g_target_cntr_offset_x = 0.0f;
    g_target_cntr_offset_y = 0.0f;
    g_target_height = 0.0f;
    g_target_width = 0.0f;
    g_target_aspect = 0.0f;
    g_target_left = 0.0f;
    g_target_right = 0.0f;
    g_target_top = 0.0f;
    g_target_bottom = 0.0f;
    g_target_detection_id = -1;
    target_detection_thresh = 0.0f;
    g_detection_class = (float)0.0;
    g_target_detection_conf = (float)0.0;

    initialized_cv_image = false;
    target_tracked = false;
    tracking = false;
    target_bounding_box = cv::Rect(0.0, 0.0, 0.0, 0.0);
    target_tracker = cv::TrackerCSRT::create();

    return true;
}

/********************************************************************************
 * Function: loop
 * Description: Determine target to be tracked and maintain identity of target
 *              from loop to loop.
 ********************************************************************************/
void Track::loop(void)
{
    identify_target();
    get_target_info();
    validate_target();
}

/********************************************************************************
 * Function: shutdown
 * Description: Cleanup code to run at the end of the program.
 ********************************************************************************/
void Track::shutdown(void)
{

}

#endif // ENABLE_CV
