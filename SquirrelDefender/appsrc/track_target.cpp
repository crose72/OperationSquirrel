#ifdef JETSON_B01

/********************************************************************************
 * @file    track_target.cpp
 * @author  Cameron Rose
 * @date    6/7/2023
 * @brief   Target tracking implementation for when the detection model
 *          fails.
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
DebugTerm TrackingData("");

bool target_tracked;
bool initialized_cv_image;
bool target_identified;
int target_detection_ID;
int target_track_ID;
float target_cntr_offset_x;
float target_cntr_offset_y;
float target_height;
float target_width;
float target_aspect;
float target_left;
float target_right;
float target_top;
float target_bottom;

cv::cuda::GpuMat gpuImage;
cv::Mat image_cv_wrapped;
cv::Ptr<cv::Tracker> target_tracker;
cv::Rect2d target_bounding_box;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/
const float center_of_frame_width = 640.0f;
const float center_of_frame_height = 360.0f;

/********************************************************************************
 * Function definitions
 ********************************************************************************/

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
 * Function: tracker_init
 * Description: Initialize the tracker by resetting the bounding box to zeros.
 ********************************************************************************/
void Track::tracker_init(cv::Ptr<cv::Tracker> &tracker, cv::Mat &image, cv::Rect2d &bounding_box)
{
    target_bounding_box = cv::Rect2d(0.0, 0.0, 0.0, 0.0);
    tracker->init(image, target_bounding_box);
}

/********************************************************************************
 * Function: tracker_update
 * Description: Update the bounding box around the tracked target.
 ********************************************************************************/
bool Track::tracker_update(cv::Ptr<cv::Tracker> &tracker, cv::Mat &image, cv::Rect2d &bounding_box)
{
    return tracker->update(image, bounding_box);
}

/********************************************************************************
 * Function: get_target_info
 * Description: Obtain the important information about the target, such as the
 *              height and location of the center of the target in the frame.
 ********************************************************************************/
void Track::get_target_info(int n)
{
    float target_center_y = (detections[n].Left + detections[n].Right) / 2.0f;
    float target_center_x = (detections[n].Bottom + detections[n].Top) / 2.0f;

    target_track_ID = detections[n].TrackID;
    target_cntr_offset_y = target_center_y - center_of_frame_width;
    target_cntr_offset_x = target_center_x - center_of_frame_height;
    target_height = detections[n].Height();
    target_width = detections[n].Width();
    target_aspect = target_width / target_height;
    target_left = detections[n].Left;
    target_right = detections[n].Right;
    target_top = detections[n].Top;
    target_bottom = detections[n].Bottom;
}

/********************************************************************************
 * Function: update_target_info
 * Description: Obtain the important information about the target, such as the
 *              height and location of the center of the target in the frame.
 ********************************************************************************/
void Track::update_target_info(void)
{
}

/********************************************************************************
 * Function: dtrmn_target
 * Description: Determine which detected object to track.
 ********************************************************************************/
void Track::dtrmn_target(void)
{
    target_detection_ID = -1;

    for (int n = 0; n < detection_count; n++)
    {
        /* A tracked object, classified as a person with some confidence level */
        if (detections[n].TrackID >= 0 && detections[n].ClassID == 1 && detections[n].Confidence > 0.5)
        {
            target_detection_ID = n;
        }
    }

    get_target_info(target_detection_ID);

    /* Target detected, tracked, and has a size greater than 0 */
    if (target_detection_ID >= 0 && target_track_ID >= 0 && target_height > 1 && target_width > 1)
    {
        target_identified = true;
    }
    else
    {
        target_identified = false;
    }
}

/********************************************************************************
 * Function: track_target
 * Description: Use the bounding box provided by the detection model as the
 *              basis for target_tracked the targeted object.
 ********************************************************************************/
void Track::track_target(void)
{
    /* Don't wrap the image from jetson inference until a valid image has been received.
       That way we know the memory has been allocaed and is ready. */
    if (valid_image_rcvd && !initialized_cv_image)
    {
        // gpuImage = cv::cuda::GpuMat(input_video_height, input_video_width, CV_8UC3);
        image_cv_wrapped = cv::Mat(input_video_height, input_video_width, CV_8UC3, image); // Directly wrap uchar3*
        initialized_cv_image = true;
    }
    else if (valid_image_rcvd && initialized_cv_image)
    {
        target_bounding_box = cv::Rect2d(target_left, target_top, target_width, target_height);
        target_tracked = tracker_update(target_tracker, image_cv_wrapped, target_bounding_box);
    }

    if (target_tracked)
    {
        // Draw the target_tracked box
        cv::rectangle(image_cv_wrapped, target_bounding_box, cv::Scalar(255, 0, 0), 2, 1);
    }
}

/********************************************************************************
 * Function: localize_target_init
 * Description: Initialize all Track target variables.  Run once at the start
 *              of the program.
 ********************************************************************************/
bool Track::init(void)
{
    target_identified = false;
    target_cntr_offset_x = 0.0f;
    target_cntr_offset_y = 0.0f;
    target_height = 0.0f;
    target_width = 0.0f;
    target_aspect = 0.0f;
    target_left = 0.0f;
    target_right = 0.0f;
    target_top = 0.0f;
    target_bottom = 0.0f;
    target_detection_ID = -1;

    target_tracker = cv::TrackerCSRT::create();
    initialized_cv_image = false;
    target_tracked = false;
    target_bounding_box = cv::Rect2d(0.0, 0.0, 0.0, 0.0);
    // Allocate memory for image before using it
    cudaMallocManaged(&image, input_video_width * input_video_height * sizeof(uchar3));
    // gpuImage.upload(image_cv_wrapped);

    return true;
}

/********************************************************************************
 * Function: localize_control_loop
 * Description: Return control parameters for the vehicle to Track a designated
 *              target at a distance.
 ********************************************************************************/
void Track::loop(void)
{
    dtrmn_target();
    track_target();
    update_target_info();
}

#endif // JETSON_B01