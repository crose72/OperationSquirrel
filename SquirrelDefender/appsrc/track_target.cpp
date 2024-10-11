#ifdef JETSON_B01

/********************************************************************************
 * @file    track_target.cpp
 * @author  Cameron Rose
 * @date    6/7/2023
 * @brief   Additional tracking implementation for when the detection model
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

int frame_count;
auto start = std::chrono::high_resolution_clock::now();
cv::cuda::GpuMat gpuImage;
cv::Mat cvImage;
cv::Ptr<cv::Tracker> tracker;
bool tracking;
bool cv_image_initialized;
cv::Rect2d bbox;

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

void tracker_init(cv::Ptr<cv::Tracker> &tracker, cv::Mat &cvImage, cv::Rect2d &bbox, bool &tracking)
{
    bbox = cv::Rect2d(0.0, 0.0, 0.0, 0.0);
    tracker->init(cvImage, bbox);
    tracking = true;
}

// Function to update tracking
bool tracker_update(cv::Ptr<cv::Tracker> &tracker, cv::Mat &cvImage, cv::Rect2d &bbox)
{
    return tracker->update(cvImage, bbox);
}

/********************************************************************************
 * Function: localize_target_init
 * Description: Initialize all Track target variables.  Run once at the start
 *              of the program.
 ********************************************************************************/
bool Track::init(void)
{
    tracker = cv::TrackerCSRT::create();
    tracking = false;
    bbox = cv::Rect2d(0.0, 0.0, 0.0, 0.0);
    // Allocate memory for image before using it
    cudaMallocManaged(&image, input_video_width * input_video_height * sizeof(uchar3));
    gpuImage.upload(cvImage);
    cv_image_initialized = false;

    return true;
}

/********************************************************************************
 * Function: localize_control_loop
 * Description: Return control parameters for the vehicle to Track a designated
 *              target at a distance.
 ********************************************************************************/
void Track::loop(void)
{
    if (valid_image_rcvd && !cv_image_initialized)
    {
        gpuImage = cv::cuda::GpuMat(input_video_height, input_video_width, CV_8UC3);
        cvImage = cv::Mat(input_video_height, input_video_width, CV_8UC3, image); // Directly wrap uchar3*
        cv_image_initialized = true;
    }
    else if (valid_image_rcvd && cv_image_initialized)
    {
        tracking = true;
        if (!tracking)
        {
            tracker_init(tracker, cvImage, bbox, tracking);
        }
        if (tracking)
        {
            bbox = cv::Rect2d(object_left, object_top, object_width, object_height);
            bool success = tracker_update(tracker, cvImage, bbox);
            if (success)
            {
                // Draw the tracking box
                cv::rectangle(cvImage, bbox, cv::Scalar(255, 0, 0), 2, 1);
            }
            else
            {
                // Tracking failed, reinitialize if needed
                tracking = false;
            }
        }
        // cv::imshow("Jetson Inference Image", cvImage);
        // cv::waitKey(1);
    }
}

#endif // JETSON_B01